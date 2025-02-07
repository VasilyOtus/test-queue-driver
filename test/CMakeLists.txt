cmake_minimum_required(VERSION 3.0.2)

project(tq VERSION 1.0.0)

find_package(GTest)

set(TEST_NAME ${PROJECT_NAME}-test)

# Build
add_executable(${TEST_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/test.cpp)

# Properties
# Properties
set_target_properties(${TEST_NAME}
	PROPERTIES
	CXX_STANDARD 17
	CXX_STANDARD_REQUIRED ON
)

target_include_directories(${TEST_NAME}
	PRIVATE ${GTEST_INCLUDE_DIRS}
)

target_link_libraries(${TEST_NAME}
	PRIVATE ${GTEST_BOTH_LIBRARIES} gtest gtest_main
)

target_compile_options(${TEST_NAME}
	PRIVATE -Wall -Wextra -pedantic -Werror
)

# Tests
enable_testing()
include(GoogleTest)
gtest_discover_tests(${TEST_NAME})
