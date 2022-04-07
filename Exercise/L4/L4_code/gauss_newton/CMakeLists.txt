cmake_minimum_required( VERSION 2.8 )
project(gaussnewton)

set( CMAKE_CXX_STANDARD 11 )

include_directories("/usr/local/include/eigen3")
find_package( OpenCV 3 REQUIRED)

include_directories(
	${OpenCV_INCLUDE_DIRS}
	)

add_executable( gaussnewton gaussnewton.cpp )
target_link_libraries( gaussnewton ${OpenCV_LIBS})
