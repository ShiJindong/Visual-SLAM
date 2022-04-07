# 第一节课习题4:  CMake练习
# 按照以下步骤输入命令行

# 首先编译文件夹 CMake_Exercise 中的 CMakeLists.txt
cd ~/.../CMake_Exercise
mkdir builb
cd build
cmake ..
make
# 文件夹CMake_Exercise/build中生成动态库libhello.so 和 可执行文件sayhello
# 执行可执行文件sayhello  =>  成功
./useHello

# 安装
sudo make install
# 安装成功，并生成FindHello.cmake文件    =>  可查看目录/usr/local/include 和 /usr/local/lib



# 测试是否可以通过find_package定位到这个库 MyHello::hello
# 文件夹test_folder中的CMakeLists.txt利用find_package(MyHello 1.0)，可成功定位到库MyHello::hello
cd ~/.../CMake_Exercise/test_folder
mkdir build
cd build
cmake ..
make
# 执行可执行文件sayhello  =>  成功
./sayhello
# 证明其他用户可以通过find_package找到我的库


