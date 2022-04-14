cmake_minimum_required( VERSION 2.8 )
project(disparity)

# 添加c++11标准支持
set( CMAKE_CXX_FLAFS "-std=c++11" )                   
# set( CMAKE_CXX_STANDARD 11)

# 寻找eigen3库
find_package( Eigen3  3.1.0 REQUIRED)
# 寻找opencv库
find_package( OpenCV 3 REQUIRED)
# 寻找pangolin库
find_package( Pangolin REQUIRED)

# 包含头文件
include_directories(
  ${Eigen3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
)

add_executable( disparity disparity.cpp )
# 链接第三方库
target_link_libraries( disparity ${OpenCV_LIBS} ${Pangolin_LIBRARIES})
