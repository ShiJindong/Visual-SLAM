cmake_minimum_required(VERSION 2.8)

project(directBA)
set(CMAKE_BUILD_TYPE "Release")
set( CMAKE_CXX_FLAGS "-O3" )

Find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

Find_Package(G2O REQUIRED)
Find_Package(Cholmod REQUIRED)
Find_package( OpenCV 3 REQUIRED )


# set(Pangolin_INCLUDE_DIRS "/usr/local/include") 

set(Sophus_INCLUDE_DIRS "/usr/local/include")    
set(Sophus_LIBRARIES "/usr/local/lib/libSophus.so")

# 添加c++ 17标准支持
set( CMAKE_CXX_STANDARD 17 )

SET(G2O_LIBS g2o_cli g2o_ext_freeglut_minimal g2o_simulator g2o_solver_slam2d_linear 
    g2o_types_icp g2o_types_slam2d g2o_types_sba g2o_types_slam3d g2o_core g2o_interface 
    g2o_solver_csparse g2o_solver_structure_only g2o_csparse_extension g2o_opengl_helper g2o_solver_dense 
    g2o_stuff g2o_types_sclam2d g2o_parser g2o_solver_pcg g2o_types_data g2o_types_sim3 cxsparse )

include_directories(
"/usr/local/include/eigen3" 
${CHOLMOD_INCLUDE_DIR} 
${Sophus_INCLUDE_DIRS} 
${G2O_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS} 

) 

message("find g2o libraries in: " ${G2O_INCLUDE_DIR})
message("find g2o lib in " ${G2O_LIBS})
message("find cholmod in " ${CHOLMOD_INCLUDE_DIR})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/OUTPUT)

add_executable(${PROJECT_NAME} directBA.cpp)

target_link_libraries(${PROJECT_NAME} ${G2O_LIBS} ${CHOLMOD_LIBRARIES} ${Sophus_LIBRARIES} ${OpenCV_LIBS} ${Pangolin_LIBRARIES})
