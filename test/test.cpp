#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <memory>
#include <functional>
#include <string>
#include <vector>
#include <cerrno>
#include <thread>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <utility>

#include <gtest/gtest.h>

namespace {

template<typename T>
	using deleted_unique_ptr = std::unique_ptr<T, std::function<void(T*)>>;

using queue_item_type = unsigned char;
constexpr size_t QUEUE_ITEM_SIZE = sizeof(queue_item_type);

using test_buf_type = std::vector<queue_item_type>;

constexpr size_t QUEUE_DEPTH_MAX = 1000;

const std::string devname = "/dev/tq";

auto write_dev(int fd, const test_buf_type& buf)
{
	auto ret = write(fd, buf.data(), buf.size() * QUEUE_ITEM_SIZE);
	return (ret < 0) ? -errno : (ret / QUEUE_ITEM_SIZE);
}

auto read_dev(int fd, test_buf_type& buf)
{
	auto ret = read(fd, buf.data(), buf.size() * QUEUE_ITEM_SIZE);
	return (ret < 0) ? -errno : (ret / QUEUE_ITEM_SIZE);
}

auto clear_dev(int fd)
{
	// Очистка очереди холостым чтением
	auto buf = test_buf_type(QUEUE_DEPTH_MAX);
	read_dev(fd, buf);
}

auto open_dev_default_mode(bool clear = true)
{
	auto fd = open(devname.c_str(), O_RDWR);
	if (fd < 0)
		return -errno;

	if (clear)
		clear_dev(fd);

	return fd;
}

auto open_dev_exclusive_mode(bool clear = true)
{
	auto fd = open(devname.c_str(), O_RDWR | O_EXCL);
	if (fd < 0)
		return -errno;

	if (clear)
		clear_dev(fd);

	return fd;
}

auto open_dev_autonomous_mode(bool /*clear*/ = true)
{
	auto fd = open(devname.c_str(), O_RDWR | O_NONBLOCK);
	return (fd < 0) ? -errno : fd;
}

auto synchronize_threads(std::mutex& mutex,
			std::condition_variable& cv,
			bool& ready)
{
	std::unique_lock lock(mutex);
	ready = true;
	lock.unlock();

	cv.notify_all();
}

void thread_routine(int fd, const test_buf_type& buf, std::mutex& mutex,
		std::condition_variable& cv, bool& ready)
{
	std::unique_lock lock(mutex);
	cv.wait(lock, [&] { return ready; });

	write_dev(fd, buf);
}

/*
 * Режим по умолчанию
 */

// Чтение из пустой очереди
TEST(tq_mode_default, read_from_empty_queue)
{
	deleted_unique_ptr<int> fd(new int, [](int* f) { close(*f); });
	*fd = open_dev_default_mode(false);
	ASSERT_GE(*fd, 0);

	auto buf = test_buf_type(QUEUE_DEPTH_MAX);

	// Очередь пустая. При попытке чтения ненулевого числа байт
	// вызов read() должен вернуть 0 (очередь пуста)
	auto ret = read_dev(*fd, buf);
	EXPECT_EQ(ret, 0);
}

// Запись с выходом за пределы максимального размера очереди
TEST(tq_mode_default, exceed_max_queue_depth)
{
	deleted_unique_ptr<int> fd(new int, [](int* f) { close(*f); });
	*fd = open_dev_default_mode();
	ASSERT_GE(*fd, 0);

	const auto ref1 = test_buf_type(QUEUE_DEPTH_MAX);
	auto ret = write_dev(*fd, ref1);
	ASSERT_EQ(ret, ref1.size());

	// В очередь записано максимальное число байт (QUEUE_DEPTH_MAX).
	// При попытке записи очередного байта вызов write() должен
	// вернуть ошибку ENOSPC
	const auto ref2 = test_buf_type(1);
	ret = write_dev(*fd, ref2);
	EXPECT_EQ(ret, -ENOSPC);
}

// Обычные запись и чтение
TEST(tq_mode_default, normal_read_write)
{
	deleted_unique_ptr<int> fd(new int, [](int* f) { close(*f); });
	*fd = open_dev_default_mode();
	ASSERT_GE(*fd, 0);

	const auto ref = test_buf_type{1, 2, 3, 4, 5, 0xff, 0xfe};
	auto buf = test_buf_type(ref.size() << 1);

	auto ret = write_dev(*fd, ref);
	ASSERT_EQ(ret, ref.size());

	// В очередь записано ref.size() байт. При попытке чтения большего
	// числа байт вызов read() должен вернуть только фактическое число
	// байт в очереди, т.е. ref.size()
	ret = read_dev(*fd, buf);
	ASSERT_EQ(ret, ref.size());

	// Записанные и считанные байты должны совпадать
	buf.resize(ref.size());
	EXPECT_EQ(buf, ref);
}

// Сохранение очереди после закрытия всех открытых дескрипторов
TEST(tq_mode_default, store_data_after_close)
{
	deleted_unique_ptr<int> fd(new int, [](int* f) { close(*f); });
	*fd = open_dev_default_mode();
	ASSERT_GE(*fd, 0);

	const auto ref = test_buf_type{1, 2, 3, 4, 5, 0xff, 0xfe};
	auto ret = write_dev(*fd, ref);
	ASSERT_EQ(ret, ref.size());

	close(*fd);

	*fd = open_dev_default_mode(false);
	ASSERT_GE(*fd, 0);

	// В очередь записано ref.size() байт, после чего дескриптор был закрыт.
	// При последующем открытии дескриптора и чтении очереди должны
	// вернуться записанные ранее байты
	auto buf = test_buf_type(ref.size());
	ret = read_dev(*fd, buf);
	ASSERT_EQ(ret, ref.size());
	EXPECT_EQ(buf, ref);
}

// Простое конкурентное использование очереди
TEST(tq_mode_default, simple_concurrent_access)
{
	deleted_unique_ptr<int> fd1(new int, [](int* f) { close(*f); });
	*fd1 = open_dev_default_mode();
	ASSERT_GE(*fd1, 0);

	deleted_unique_ptr<int> fd2(new int, [](int* f) { close(*f); });
	*fd2 = open_dev_default_mode(false);
	ASSERT_GE(*fd2, 0);

	const auto ref1 = test_buf_type{1, 2, 3};
	auto ret1 = write_dev(*fd1, ref1);
	ASSERT_EQ(ret1, ref1.size());

	const auto ref2 = test_buf_type{0xff, 0xfe, 0xfd, 0xfc, 0xfb, 0xfa};
	auto ret2 = write_dev(*fd2, ref2);
	ASSERT_EQ(ret2, ref2.size());

	deleted_unique_ptr<int> fd3(new int, [](int* f) { close(*f); });
	*fd3 = open_dev_default_mode(false);
	ASSERT_GE(*fd3, 0);

	auto ref3 = test_buf_type();
	ref3.insert(ref3.end(), ref1.begin(), ref1.end());
	ref3.insert(ref3.end(), ref2.begin(), ref2.end());

	// Произведено 2 последовательные записи в разные дескрипторы. При
	// чтении из 3-его дескриптора должен вернуться результат
	// конкатенации 2-х операций записи
	auto buf = test_buf_type(ref3.size());
	auto ret = read_dev(*fd3, buf);
	ASSERT_EQ(ret, ref3.size());
	EXPECT_EQ(buf, ref3);
}

// Конкурентное использование очереди
TEST(tq_mode_default, threaded_concurrent_access)
{
	deleted_unique_ptr<int> fd1(new int, [](int* f) { close(*f); });
	*fd1 = open_dev_default_mode();
	ASSERT_GE(*fd1, 0);

	deleted_unique_ptr<int> fd2(new int, [](int* f) { close(*f); });
	*fd2 = open_dev_default_mode(false);
	ASSERT_GE(*fd2, 0);

	std::mutex mutex;
	std::condition_variable cv;
	bool ready = false;

	const auto ref1 = test_buf_type(QUEUE_DEPTH_MAX >> 1, 0x55);
	const auto ref2 = test_buf_type(QUEUE_DEPTH_MAX >> 1, 0x55);

	std::thread thread1(thread_routine, *fd1, std::ref(ref1),
			std::ref(mutex), std::ref(cv), std::ref(ready));
	std::thread thread2(thread_routine, *fd2, std::ref(ref2),
			std::ref(mutex), std::ref(cv), std::ref(ready));

	std::this_thread::sleep_for(std::chrono::milliseconds(100));
	synchronize_threads(mutex, cv, ready);

	thread1.join();
	thread2.join();

	deleted_unique_ptr<int> fd3(new int, [](int* f) { close(*f); });
	*fd3 = open_dev_default_mode(false);
	ASSERT_GE(*fd3, 0);

	auto ref3 = test_buf_type();
	ref3.insert(ref3.end(), ref1.begin(), ref1.end());
	ref3.insert(ref3.end(), ref2.begin(), ref2.end());

	// Произведено 2 условно одновременные записи в разные дескрипторы.
	// При чтении из 3-его дескриптора должен вернуться результат
	// конкатенации 2-х операций записи
	auto buf = test_buf_type(ref3.size());
	auto ret = read_dev(*fd3, buf);
	ASSERT_EQ(ret, ref3.size());
	EXPECT_EQ(buf, ref3);
}

// Эксклюзивный доступ к очереди после доступа по умолчанию
TEST(tq_mode_default, exclusive_access_after_default_open)
{
	deleted_unique_ptr<int> fd1(new int, [](int* f) { close(*f); });
	*fd1 = open_dev_default_mode();
	ASSERT_GE(*fd1, 0);

	// Устройство открыто в режиме по умолчанию. Попытка открытия в
	// одиночном режиме должна завершиться с ошибкой EBUSY
	deleted_unique_ptr<int> fd2(new int, [](int* f) { close(*f); });
	*fd2 = open_dev_exclusive_mode(false);
	EXPECT_EQ(*fd2, -EBUSY);
}

// Автономный доступ к очереди после доступа по умолчанию
TEST(tq_mode_default, autonomous_access_after_default_open)
{
	deleted_unique_ptr<int> fd1(new int, [](int* f) { close(*f); });
	*fd1 = open_dev_default_mode();
	ASSERT_GE(*fd1, 0);

	// Устройство открыто в режиме по умолчанию. Попытка открытия в
	// параллельном режиме должна завершиться успешно
	deleted_unique_ptr<int> fd2(new int, [](int* f) { close(*f); });
	*fd2 = open_dev_autonomous_mode(false);
	EXPECT_GE(*fd2, 0);
}

/*
 * Одиночный режим
 */

// Эксклюзивный доступ к очереди после эксклюзивного доступа
TEST(tq_mode_exclusive, exclusive_access_after_exclusive_open)
{
	deleted_unique_ptr<int> fd1(new int, [](int* f) { close(*f); });
	*fd1 = open_dev_exclusive_mode();
	ASSERT_GE(*fd1, 0);

	// Устройство открыто в одиночном режиме. Попытка открытия устройства
	// в одиночном режиме должна завершиться с ошибкой EBUSY
	deleted_unique_ptr<int> fd2(new int, [](int* f) { close(*f); });
	*fd2 = open_dev_exclusive_mode(false);
	EXPECT_EQ(*fd2, -EBUSY);
}

// Конкурентный доступ к очереди после эксклюзивного доступа
TEST(tq_mode_exclusive, concurrent_access_after_exclusive_open)
{
	deleted_unique_ptr<int> fd1(new int, [](int* f) { close(*f); });
	*fd1 = open_dev_exclusive_mode();
	ASSERT_GE(*fd1, 0);

	// Устройство открыто в одиночном режиме. Попытка открытия устройства
	// в режиме по умолчанию должна завершиться с ошибкой EBUSY
	deleted_unique_ptr<int> fd2(new int, [](int* f) { close(*f); });
	*fd2 = open_dev_default_mode(false);
	EXPECT_EQ(*fd2, -EBUSY);
}

// Автономный доступ к очереди после эксклюзивного доступа
TEST(tq_mode_exclusive, autonomous_access_after_exclusive_open)
{
	deleted_unique_ptr<int> fd1(new int, [](int* f) { close(*f); });
	*fd1 = open_dev_exclusive_mode();
	ASSERT_GE(*fd1, 0);

	// Устройство открыто в одиночном режиме. Попытка открытия устройства
	// в параллельном режиме должна завершиться успешно
	deleted_unique_ptr<int> fd2(new int, [](int* f) { close(*f); });
	*fd2 = open_dev_autonomous_mode(false);
	EXPECT_GE(*fd2, 0);
}

/*
 * Параллельный режим
 */

// Автономный доступ к очереди
TEST(tq_mode_autonomous, autonomous_access)
{
	deleted_unique_ptr<int> fd1(new int, [](int* f) { close(*f); });
	*fd1 = open_dev_autonomous_mode();
	ASSERT_GE(*fd1, 0);

	deleted_unique_ptr<int> fd2(new int, [](int* f) { close(*f); });
	*fd2 = open_dev_autonomous_mode(false);
	EXPECT_GE(*fd2, 0);

	const auto ref1 = test_buf_type{1, 2, 3, 4};
	auto ret1 = write_dev(*fd1, ref1);
	ASSERT_EQ(ret1, ref1.size());

	const auto ref2 = test_buf_type{0xff, 0xfe, 0xfd, 0xfc, 0xfb, 0xfa};
	auto ret2 = write_dev(*fd2, ref2);
	ASSERT_EQ(ret2, ref2.size());

	// Произведено 2 записи в параллельном режиме. Данные в очередях
	// должны быть независимы друг от друга
	auto buf1 = test_buf_type(ref1.size() << 1);
	ret1 = read_dev(*fd1, buf1);

	auto buf2 = test_buf_type(ref2.size() << 1);
	ret2 = read_dev(*fd2, buf2);
	ASSERT_EQ(ret1, ref1.size());
	ASSERT_EQ(ret2, ref2.size());

	// Записанные и считанные байты должны совпадать
	buf1.resize(ref1.size());
	buf2.resize(ref2.size());
	EXPECT_EQ(buf1, ref1);
	EXPECT_EQ(buf2, ref2);
}

// Пустая очередь в параллельном режиме после закрытия дескриптора
TEST(tq_mode_autonomous, empty_queue_after_reopen)
{
	deleted_unique_ptr<int> fd(new int, [](int* f) { close(*f); });
	*fd = open_dev_autonomous_mode();
	ASSERT_GE(*fd, 0);

	const auto ref = test_buf_type{1, 2, 3, 4, 5};
	auto ret = write_dev(*fd, ref);
	ASSERT_EQ(ret, ref.size());

	close(*fd);

	*fd = open_dev_autonomous_mode(false);
	ASSERT_GE(*fd, 0);

	// После закрытия дескриптора в параллельном режиме очередь очищается.
	// Повторное открытие дескриптора в параллельном режиме приводит к
	// созданию новой пустой очереди
	auto buf = test_buf_type(ref.size() << 1);
	ret = read_dev(*fd, buf);
	EXPECT_EQ(ret, 0);
}

// Сохранение очереди в одиночном режиме после автономного доступа
TEST(tq_mode_autonomous, store_data_after_autonomous_access)
{
	deleted_unique_ptr<int> fd1(new int, [](int* f) { close(*f); });
	*fd1 = open_dev_autonomous_mode();
	ASSERT_GE(*fd1, 0);

	deleted_unique_ptr<int> fd2(new int, [](int* f) { close(*f); });
	*fd2 = open_dev_exclusive_mode(false);
	ASSERT_GE(*fd2, 0);

	const auto ref1 = test_buf_type{1, 2, 3, 4, 5};
	auto ret1 = write_dev(*fd1, ref1);
	ASSERT_EQ(ret1, ref1.size());

	const auto ref2 = test_buf_type{0xff, 0xfe, 0xfd, 0xfc, 0xfb, 0xfa};
	auto ret2 = write_dev(*fd2, ref2);
	ASSERT_EQ(ret2, ref2.size());

	close(*fd1);
	close(*fd2);

	*fd1 = open_dev_autonomous_mode(false);
	ASSERT_GE(*fd1, 0);

	*fd2 = open_dev_exclusive_mode(false);
	ASSERT_GE(*fd2, 0);

	// Произведено 2 записи: в параллельном и в одиночном режиме. Данные в
	// очереди одиночного режима должны сохраниться после переоткрытия
	// дескрипторов, а очередь параллельного режима должна быть пустой
	auto buf1 = test_buf_type(ref1.size() << 1);
	ret1 = read_dev(*fd1, buf1);

	auto buf2 = test_buf_type(ref2.size() << 1);
	ret2 = read_dev(*fd2, buf2);
	ASSERT_EQ(ret1, 0);
	ASSERT_EQ(ret2, ref2.size());

	// Записанные и считанные байты в очереди одиночного режима должны
	// совпадать
	buf2.resize(ref2.size());
	EXPECT_EQ(buf2, ref2);
}

};

int main(int argc, char* argv[])
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
