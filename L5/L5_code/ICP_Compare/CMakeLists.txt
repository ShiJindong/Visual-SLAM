cmake_minimum_required( VERSION 2.8 )
project( ICP_Compare )

# 添加c++ 17标准支持
set( CMAKE_CXX_STANDARD 17 )     
set( CMAKE_BUILD_TYPE "Debug")

# 添加Eigen头文件
include_directories( "/usr/local/include/eigen3" )

find_package(Pangolin REQUIRED)

set(Sophus_INCLUDE_DIRS "/usr/local/include")     
set(Sophus_LIBRARIES "/usr/local/lib/libSophus.so")


include_directories(
${Pangolin_INCLUDE_DIRS}
${Sophus_INCLUDE_DIRS} 
)

add_executable( ICP_Compare ICP_Compare.cpp )
target_link_libraries( ICP_Compare ${Sophus_LIBRARIES} ${Pangolin_LIBRARIES} )
