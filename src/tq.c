// SPDX-License-Identifier: GPL-2.0-only
/*
 * Test queue driver
 *
 * Copyright (C) 2025 Vasily Vinogradov <wmdlr@yandex.ru>
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/miscdevice.h>
#include <linux/fcntl.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/mutex.h>
#include <linux/list.h>
#include <linux/uaccess.h>

#define TQ_DRIVER_NAME		"tq"
#define TQ_DRIVER_VERSION	"1.0.0"

/* Driver modes */
#define TQ_MODE_DEFAULT		0	/* concurrent I/O */
#define TQ_MODE_EXCLUSIVE	BIT(0)	/* exclusive access */
#define TQ_MODE_INDIVIDUAL	BIT(1)	/* individual I/O */

/* Max queue depth */
#define TQ_QUEUE_DEPTH_MAX	1000

struct tq_data_item {
	struct list_head	node;
	unsigned char		data;
};

struct tq_device {
	struct mutex		lock;
	struct list_head	queue;
	size_t			queue_depth;
};

static struct kmem_cache *tq_data_cache;

static DEFINE_SPINLOCK(tq_state_lock);
static size_t tq_special_open_cnt;	/* count of concurrent opens */
static int tq_open_mode;		/* open mode */

static struct tq_device *tq_dev;

static struct tq_device *tq_device_create(void)
{
	struct tq_device *dev;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (dev == NULL)
		return ERR_PTR(-ENOMEM);

	mutex_init(&dev->lock);
	INIT_LIST_HEAD(&dev->queue);

	return dev;
}

static void tq_device_destroy(struct tq_device *dev)
{
	struct tq_data_item *item, *tmp_item;

	list_for_each_entry_safe(item, tmp_item, &dev->queue, node) {
		list_del(&item->node);
		kmem_cache_free(tq_data_cache, item);
	}

	mutex_destroy(&dev->lock);
	kfree(dev);
}

static ssize_t tq_read(struct file *filp, char __user *buf, size_t count,
		       loff_t *ppos)
{
	struct tq_device *dev = filp->private_data;
	struct tq_data_item *item;
	typeof(item->data) *ptr = (void *)buf;
	ssize_t ret;

	if (!access_ok(buf, count))
		return -EFAULT;

	mutex_lock(&dev->lock);

	count = min_t(size_t, count / sizeof(*ptr), dev->queue_depth);
	ret = count * sizeof(*ptr);

	while (count--) {
		item = list_first_entry(&dev->queue, typeof(*item), node);

		if (__put_user(item->data, ptr++)) {
			ret = -EFAULT;
			goto out;
		}

		list_del(&item->node);
		kmem_cache_free(tq_data_cache, item);
		--dev->queue_depth;
	}

out:
	mutex_unlock(&dev->lock);
	return ret;
}

static ssize_t tq_write(struct file *filp, const char __user *buf, size_t count,
			loff_t *ppos)
{
	struct tq_device *dev = filp->private_data;
	struct tq_data_item *item;
	const typeof(item->data) *ptr = (void *)buf;
	ssize_t ret;

	if (!access_ok(buf, count))
		return -EFAULT;

	ret = rounddown(count, sizeof(*ptr));
	count = ret / sizeof(*ptr);

	mutex_lock(&dev->lock);

	if (dev->queue_depth + count > TQ_QUEUE_DEPTH_MAX) {
		ret = -ENOSPC;
		goto out;
	}

	while (count--) {
		item = kmem_cache_alloc(tq_data_cache, GFP_KERNEL);
		if (item == NULL) {
			ret = -ENOMEM;
			goto out;
		}

		if (__get_user(item->data, ptr++)) {
			ret = -EFAULT;
			goto out;
		}

		list_add_tail(&item->node, &dev->queue);
		++dev->queue_depth;
	}

out:
	mutex_unlock(&dev->lock);
	return ret;
}

static int tq_open(struct inode *inode, struct file *filp)
{
	struct tq_device *dev;
	int ret;

	ret = nonseekable_open(inode, filp);
	if (ret)
		return ret;

	/* Individual mode */
	if (filp->f_flags & O_NONBLOCK) {
		dev = tq_device_create();
		if (IS_ERR(dev))
			return PTR_ERR(dev);

		filp->private_data = dev;
		return 0;
	}

	spin_lock(&tq_state_lock);

	/* Prevent multiple readers/writers in exclusive mode */
	if ((tq_special_open_cnt && (filp->f_flags & O_EXCL)) ||
	    (tq_open_mode & TQ_MODE_EXCLUSIVE)) {
		spin_unlock(&tq_state_lock);
		return -EBUSY;
	}

	if (filp->f_flags & O_EXCL)
		tq_open_mode |= TQ_MODE_EXCLUSIVE;

	++tq_special_open_cnt;

	filp->private_data = tq_dev;

	spin_unlock(&tq_state_lock);

	return 0;
}

static int tq_release(struct inode *inode, struct file *filp)
{
	struct tq_device *dev = filp->private_data;

	if (filp->f_flags & O_NONBLOCK) {
		tq_device_destroy(dev);
		return 0;
	}

	spin_lock(&tq_state_lock);

	--tq_special_open_cnt;

	/* If only one instance is open, clear the EXCLUSIVE bit */
	if (tq_open_mode & TQ_MODE_EXCLUSIVE)
		tq_open_mode &= ~TQ_MODE_EXCLUSIVE;

	spin_unlock(&tq_state_lock);

	return 0;
}

static const struct file_operations tq_fops = {
	.owner		= THIS_MODULE,
	.read		= tq_read,
	.write		= tq_write,
	.open		= tq_open,
	.release	= tq_release,
};

static struct miscdevice tq_misc = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= TQ_DRIVER_NAME,
	.fops	= &tq_fops,
	.mode	= 0666,
};

static int __init tq_module_init(void)
{
	int ret;

	tq_data_cache = kmem_cache_create(TQ_DRIVER_NAME,
					  sizeof(struct tq_data_item), 0,
					  0, NULL);
	if (tq_data_cache == NULL) {
		ret = -ENOMEM;
		pr_err("%s: failed to create memory cache: %d\n",
		       TQ_DRIVER_NAME, ret);
		goto kmem_cache_create_failed;
	}

	tq_dev = tq_device_create();
	if (IS_ERR(tq_dev)) {
		ret = PTR_ERR(tq_dev);
		goto tq_device_create_failed;
	}

	ret = misc_register(&tq_misc);
	if (ret) {
		pr_err("%s: failed to register misc device: %d\n",
		       TQ_DRIVER_NAME, ret);
		goto misc_register_failed;
	}

	pr_info("%s: Test queue driver v%s\n",
		TQ_DRIVER_NAME, TQ_DRIVER_VERSION);
	return 0;

misc_register_failed:
	tq_device_destroy(tq_dev);

tq_device_create_failed:
	kmem_cache_destroy(tq_data_cache);

kmem_cache_create_failed:
	return ret;
}

static void __exit tq_module_exit(void)
{
	misc_deregister(&tq_misc);
	tq_device_destroy(tq_dev);
	kmem_cache_destroy(tq_data_cache);
}

module_init(tq_module_init);
module_exit(tq_module_exit);

MODULE_DESCRIPTION("Test queue driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(TQ_DRIVER_VERSION);
MODULE_AUTHOR("Vasily Vinogradov <wmdlr@yandex.ru>");
