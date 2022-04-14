project(useMyHelloTest)
cmake_minimum_required(VERSION 3.5)

find_package(MyHello 1.0)

# General set up
SET(CMAKE_BUILD_TYPE “Release”)
SET(CMAKE_CXX_STANDARD 11)

add_executable(sayhello useHello.cpp)
target_link_libraries(sayhello MyHello::hello)
