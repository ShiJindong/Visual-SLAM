cmake_minimum_required(VERSION 2.6)
project(qr_cholesky_test)

# 添加Eigen头文件
include_directories( "/usr/include/eigen3" )

add_executable(qr_cholesky_test qr_cholesky_test.cpp)

install(TARGETS qr_cholesky_test RUNTIME DESTINATION bin)
