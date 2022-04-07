# cmake 最低版本需求
cmake_minimum_required(VERSION 3.0)
# 工程名称
project(useMyHello VERSION 1.0)

# General set up
SET(CMAKE_BUILD_TYPE “Release”)
SET(CMAKE_CXX_STANDARD 11)

# generate a dynamic library called hello
add_library(hello SHARED hello.cpp)
# link from the header datei in folder include
target_include_directories(hello PRIVATE  ${CMAKE_SOURCE_DIR}/include)
set_target_properties(hello PROPERTIES PUBLIC_HEADER ${CMAKE_SOURCE_DIR}/include/hello.h)
# Alternativ: add_library(hello SHARED src/hello.cpp include/hello.h)

# 生成可执行文件
add_executable(sayhello useHello.cpp)
target_link_libraries(sayhello hello)
target_include_directories(sayhello PRIVATE ${CMAKE_SOURCE_DIR}/include)

# 改变安装路径可以使用下面的语句，Unix: "/usr/local", Windows: "c:/Program Files/${PROJECT_NAME}" 为默认值，所有的DESTINATION均以此目录为根目录
# set(CMAKE_INSTALL_PREFIX ${CMAKE_BINARY_DIR}/hello)

# 声明库的版本信息
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
        FindHelloConfigVersion.cmake
        VERSION ${PACKAGE_VERSION}
        COMPATIBILITY AnyNewerVersion
        )

# 将库文件，可执行文件，头文件安装到指定目录
install(TARGETS hello sayhello
        EXPORT MyHelloTargets
	# 设置动态库安装路径为LIBRARY DESTINATION lib， 设置静态库安装路径为ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
	# 设置可执行文件安装路径
        RUNTIME DESTINATION bin
	# 设置头文件安装路径 
        PUBLIC_HEADER DESTINATION include
	# 也可以写上${CMAKE_INSTALL_PREFIX}/，默认安装目录为/usr/local
        )

# 将库的相关信息输出为FindHello.cmake，供find_package查找
install(EXPORT MyHelloTargets
        FILE FindHello.cmake
        NAMESPACE MyHello::
        DESTINATION lib/cmake/MyHello
        )

# 配置生成MyHelloTargets.cmake文件，并一同安装到${CMAKE_INSTALL_PREFIX}/lib/cmake/MyHello目录中
configure_file(FindHelloConfig.cmake.in FindHelloConfig.cmake @ONLY)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/FindHelloConfig.cmake"
                "${CMAKE_CURRENT_BINARY_DIR}/FindHelloConfigVersion.cmake"
        DESTINATION lib/cmake/MyHello
        )
