cmake_minimum_required (VERSION 3.0)

project ("gtest_program")

# GTest
enable_testing()

find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

# Add source to this project's executable.
add_executable (CPtest gtest_sum.cpp)

# TODO: Add tests and install targets if needed.
target_link_libraries(CPtest PRIVATE GTest::gtest GTest::gtest_main)
# target_link_libraries(CPtest GTest::gtest)

add_test(CPtest CPtest)








