cmake_minimum_required(VERSION 2.6)
project(geometry_exercise)

# 添加Eigen头文件
include_directories( "/usr/include/eigen3" )

add_executable( geometry_exercise geometry_exercise.cpp )
