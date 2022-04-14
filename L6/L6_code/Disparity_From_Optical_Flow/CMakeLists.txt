cmake_minimum_required( VERSION 3.0 )
project(disparity_from_optical_flow)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE "Release")

include_directories(/usr/local/include/eigen3)
find_package(OpenCV 3 REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/OUTPUT)

add_executable(disparity_from_optical_flow disparity_from_optical_flow.cpp)
target_link_libraries(disparity_from_optical_flow ${OpenCV_LIBS} tbb)


