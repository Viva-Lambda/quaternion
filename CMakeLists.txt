cmake_minimum_required (VERSION 3.0.2)
project("quaternion")

# include ctest
include(CTest)

#
enable_testing()

set (CMAKE_BUILD_TYPE "Release")

set (CMAKE_CXX_FLAGS "-std=c++03")
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -ggdb -Wall -Wextra -ldl")


# include test suite
include_directories("./include/")

# test dir
set (TEST_DIR "${PROJECT_SOURCE_DIR}/tests")


# glob all test files
add_executable(test.out 
    "${TEST_DIR}/test_quaternion.cpp"
    "${PROJECT_SOURCE_DIR}/test.c"
    )
