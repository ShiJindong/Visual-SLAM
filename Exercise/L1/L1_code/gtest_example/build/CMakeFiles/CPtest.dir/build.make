# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.17

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Disable VCS-based implicit rules.
% : %,v


# Disable VCS-based implicit rules.
% : RCS/%


# Disable VCS-based implicit rules.
% : RCS/%,v


# Disable VCS-based implicit rules.
% : SCCS/s.%


# Disable VCS-based implicit rules.
% : s.%


.SUFFIXES: .hpux_make_needs_suffix_list


# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/jindong/SLAM/Chap1/L1_code/gtest_example

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jindong/SLAM/Chap1/L1_code/gtest_example/build

# Include any dependencies generated for this target.
include CMakeFiles/CPtest.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/CPtest.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/CPtest.dir/flags.make

CMakeFiles/CPtest.dir/gtest_sum.cpp.o: CMakeFiles/CPtest.dir/flags.make
CMakeFiles/CPtest.dir/gtest_sum.cpp.o: ../gtest_sum.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jindong/SLAM/Chap1/L1_code/gtest_example/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/CPtest.dir/gtest_sum.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/CPtest.dir/gtest_sum.cpp.o -c /home/jindong/SLAM/Chap1/L1_code/gtest_example/gtest_sum.cpp

CMakeFiles/CPtest.dir/gtest_sum.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/CPtest.dir/gtest_sum.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jindong/SLAM/Chap1/L1_code/gtest_example/gtest_sum.cpp > CMakeFiles/CPtest.dir/gtest_sum.cpp.i

CMakeFiles/CPtest.dir/gtest_sum.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/CPtest.dir/gtest_sum.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jindong/SLAM/Chap1/L1_code/gtest_example/gtest_sum.cpp -o CMakeFiles/CPtest.dir/gtest_sum.cpp.s

# Object files for target CPtest
CPtest_OBJECTS = \
"CMakeFiles/CPtest.dir/gtest_sum.cpp.o"

# External object files for target CPtest
CPtest_EXTERNAL_OBJECTS =

CPtest: CMakeFiles/CPtest.dir/gtest_sum.cpp.o
CPtest: CMakeFiles/CPtest.dir/build.make
CPtest: /usr/local/lib/libgtest_main.a
CPtest: /usr/local/lib/libgtest.a
CPtest: CMakeFiles/CPtest.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jindong/SLAM/Chap1/L1_code/gtest_example/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable CPtest"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/CPtest.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/CPtest.dir/build: CPtest

.PHONY : CMakeFiles/CPtest.dir/build

CMakeFiles/CPtest.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/CPtest.dir/cmake_clean.cmake
.PHONY : CMakeFiles/CPtest.dir/clean

CMakeFiles/CPtest.dir/depend:
	cd /home/jindong/SLAM/Chap1/L1_code/gtest_example/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jindong/SLAM/Chap1/L1_code/gtest_example /home/jindong/SLAM/Chap1/L1_code/gtest_example /home/jindong/SLAM/Chap1/L1_code/gtest_example/build /home/jindong/SLAM/Chap1/L1_code/gtest_example/build /home/jindong/SLAM/Chap1/L1_code/gtest_example/build/CMakeFiles/CPtest.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/CPtest.dir/depend
