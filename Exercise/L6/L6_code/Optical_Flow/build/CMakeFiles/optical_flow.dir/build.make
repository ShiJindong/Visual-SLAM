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
CMAKE_SOURCE_DIR = /home/jindong/SLAM/Chap6/L6_code/Optical_Flow

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jindong/SLAM/Chap6/L6_code/Optical_Flow/build

# Include any dependencies generated for this target.
include CMakeFiles/optical_flow.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/optical_flow.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/optical_flow.dir/flags.make

CMakeFiles/optical_flow.dir/optical_flow.cpp.o: CMakeFiles/optical_flow.dir/flags.make
CMakeFiles/optical_flow.dir/optical_flow.cpp.o: ../optical_flow.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jindong/SLAM/Chap6/L6_code/Optical_Flow/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/optical_flow.dir/optical_flow.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/optical_flow.dir/optical_flow.cpp.o -c /home/jindong/SLAM/Chap6/L6_code/Optical_Flow/optical_flow.cpp

CMakeFiles/optical_flow.dir/optical_flow.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/optical_flow.dir/optical_flow.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jindong/SLAM/Chap6/L6_code/Optical_Flow/optical_flow.cpp > CMakeFiles/optical_flow.dir/optical_flow.cpp.i

CMakeFiles/optical_flow.dir/optical_flow.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/optical_flow.dir/optical_flow.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jindong/SLAM/Chap6/L6_code/Optical_Flow/optical_flow.cpp -o CMakeFiles/optical_flow.dir/optical_flow.cpp.s

# Object files for target optical_flow
optical_flow_OBJECTS = \
"CMakeFiles/optical_flow.dir/optical_flow.cpp.o"

# External object files for target optical_flow
optical_flow_EXTERNAL_OBJECTS =

../OUTPUT/optical_flow: CMakeFiles/optical_flow.dir/optical_flow.cpp.o
../OUTPUT/optical_flow: CMakeFiles/optical_flow.dir/build.make
../OUTPUT/optical_flow: /usr/local/lib/libopencv_dnn.so.3.4.15
../OUTPUT/optical_flow: /usr/local/lib/libopencv_highgui.so.3.4.15
../OUTPUT/optical_flow: /usr/local/lib/libopencv_ml.so.3.4.15
../OUTPUT/optical_flow: /usr/local/lib/libopencv_objdetect.so.3.4.15
../OUTPUT/optical_flow: /usr/local/lib/libopencv_shape.so.3.4.15
../OUTPUT/optical_flow: /usr/local/lib/libopencv_stitching.so.3.4.15
../OUTPUT/optical_flow: /usr/local/lib/libopencv_superres.so.3.4.15
../OUTPUT/optical_flow: /usr/local/lib/libopencv_videostab.so.3.4.15
../OUTPUT/optical_flow: /usr/local/lib/libopencv_viz.so.3.4.15
../OUTPUT/optical_flow: /usr/local/lib/libopencv_calib3d.so.3.4.15
../OUTPUT/optical_flow: /usr/local/lib/libopencv_features2d.so.3.4.15
../OUTPUT/optical_flow: /usr/local/lib/libopencv_flann.so.3.4.15
../OUTPUT/optical_flow: /usr/local/lib/libopencv_photo.so.3.4.15
../OUTPUT/optical_flow: /usr/local/lib/libopencv_video.so.3.4.15
../OUTPUT/optical_flow: /usr/local/lib/libopencv_videoio.so.3.4.15
../OUTPUT/optical_flow: /usr/local/lib/libopencv_imgcodecs.so.3.4.15
../OUTPUT/optical_flow: /usr/local/lib/libopencv_imgproc.so.3.4.15
../OUTPUT/optical_flow: /usr/local/lib/libopencv_core.so.3.4.15
../OUTPUT/optical_flow: CMakeFiles/optical_flow.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jindong/SLAM/Chap6/L6_code/Optical_Flow/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../OUTPUT/optical_flow"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/optical_flow.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/optical_flow.dir/build: ../OUTPUT/optical_flow

.PHONY : CMakeFiles/optical_flow.dir/build

CMakeFiles/optical_flow.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/optical_flow.dir/cmake_clean.cmake
.PHONY : CMakeFiles/optical_flow.dir/clean

CMakeFiles/optical_flow.dir/depend:
	cd /home/jindong/SLAM/Chap6/L6_code/Optical_Flow/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jindong/SLAM/Chap6/L6_code/Optical_Flow /home/jindong/SLAM/Chap6/L6_code/Optical_Flow /home/jindong/SLAM/Chap6/L6_code/Optical_Flow/build /home/jindong/SLAM/Chap6/L6_code/Optical_Flow/build /home/jindong/SLAM/Chap6/L6_code/Optical_Flow/build/CMakeFiles/optical_flow.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/optical_flow.dir/depend

