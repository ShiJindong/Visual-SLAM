cmake_minimum_required( VERSION 2.8 )
project( computeORB )

# 添加c++ 17标准支持
set( CMAKE_CXX_STANDARD 17 )
set( CMAKE_CXX_STANDARD_REQUIRED ON )
set( CMAKE_BUILD_TYPE "Debug")

# 寻找OpenCV库
find_package( OpenCV 3 REQUIRED )     # 搜索 OpenCV 3.0 以上版本
# 寻找tbb库
find_package( TBB COMPONENTS tbbmalloc tbbmalloc_proxy tbb_preview )
# 添加头文件
include_directories( ${OpenCV_INCLUDE_DIRS} )
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/OUTPUT)


add_executable( computeORB computeORB.cpp )
# 链接OpenCV库
target_link_libraries( computeORB ${OpenCV_LIBS} TBB::tbb)
