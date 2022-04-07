# cmake 最低版本需求
cmake_minimum_required(VERSION 3.0)
# 工程名称
project(useMyHello VERSION 1.0)

# General set up
SET(CMAKE_BUILD_TYPE “Release”)
SET(CMAKE_CXX_STANDARD 11)

find_package(gflags REQUIRED)
include_directories(${gflags_INCLUDE_DIR})

# generate a dynamic library called hello
add_library(hello SHARED hello.cpp)
# link from the header datei in folder include
target_include_directories(hello PRIVATE  ${CMAKE_SOURCE_DIR}/include)
set_target_properties(hello PROPERTIES PUBLIC_HEADER ${CMAKE_SOURCE_DIR}/include/hello.h)


add_executable(sayhello useHello.cpp)
target_link_libraries(sayhello hello gflags)





