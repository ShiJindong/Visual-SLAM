cmake_minimum_required( VERSION 2.8)
project(direct_method)

IF(NOT DEFINED ${CMAKE_BUILD_TYPE})
    SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE(STATUS "CMAKE_BUILD_TYPE IS ${CMAKE_BUILD_TYPE}")

# 添加c++ 17标准支持
set( CMAKE_CXX_STANDARD 17 )

# 寻找OpenCV库
find_package( OpenCV 3 REQUIRED )     # 搜索 OpenCV 3.0 以上版本

# 添加头文件
include_directories( ${OpenCV_INCLUDE_DIRS} "/usr/local/include/eigen3" )

# 添加sophus头文件
include_directories( "/usr/local/include" )
set(Sophus_LIBRARIES "/usr/local/lib/libSophus.so")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/OUTPUT)

add_executable( direct_method direct_method.cpp )
# 链接OpenCV库
target_link_libraries( direct_method ${OpenCV_LIBS} ${Sophus_LIBRARIES} tbb)





