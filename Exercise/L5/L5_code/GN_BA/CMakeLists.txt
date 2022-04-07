cmake_minimum_required( VERSION 2.8 )
project( GN_BA )

# 添加c++ 11标准支持
set( CMAKE_CXX_STANDARD 11 )     
set( CMAKE_BUILD_TYPE "Debug")

# 添加Eigen头文件
include_directories( "/usr/local/include/eigen3" )

set(Sophus_INCLUDE_DIRS "/usr/local/include")     
set(Sophus_LIBRARIES "/usr/local/lib/libSophus.so")


include_directories(
${Sophus_INCLUDE_DIRS} 
)

add_executable( GN_BA GN_BA.cpp )
target_link_libraries( GN_BA ${Sophus_LIBRARIES} )
