cmake_minimum_required( VERSION 2.8 )
project( draw_trajectory )

# 添加Eigen头文件
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)

set(Sophus_INCLUDE_DIRS "/usr/local/include")      #set(Sophus_INCLUDE_DIRS "/usr/local/include/sophus") 
set(Sophus_LIBRARIES "/usr/local/lib/libSophus.so")

include_directories(
${Eigen3_INCLUDE_DIRS}
${Pangolin_INCLUDE_DIRS}
${Sophus_INCLUDE_DIRS} 
)

add_executable( draw_trajectory draw_trajectory.cpp )
target_link_libraries( draw_trajectory ${Sophus_LIBRARIES} ${Pangolin_LIBRARIES} )
