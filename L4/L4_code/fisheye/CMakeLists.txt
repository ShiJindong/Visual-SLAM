cmake_minimum_required( VERSION 2.8 )
project( fisheye )

# 添加c++ 11标准支持
set( CMAKE_CXX_FLAGS "-std=c++11" )

# 寻找OpenCV库
find_package( OpenCV 3 REQUIRED )     # 搜索 OpenCV 3.0 以上版本
# 添加头文件
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable( fisheye fisheye.cpp )
# 链接OpenCV库
target_link_libraries( fisheye ${OpenCV_LIBS} )
