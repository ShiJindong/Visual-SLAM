cmake_minimum_required( VERSION 2.8 )
project( E2Rt )

# 添加c++ 11标准支持
set( CMAKE_CXX_FLAGS "-std=c++11" )
set( CMAKE_BUILD_TYPE "Debug")

# 添加Eigen头文件
# find_package(Eigen3 3.1.0 REQUIRED)
include_directories( "/usr/local/include/eigen3/" )

set(Sophus_INCLUDE_DIRS "/usr/local/include")     
set(Sophus_LIBRARIES "/usr/local/lib/libSophus.so")


include_directories(
${Sophus_INCLUDE_DIRS} 
)

add_executable( E2Rt E2Rt.cpp )
target_link_libraries( E2Rt ${Sophus_LIBRARIES} )
