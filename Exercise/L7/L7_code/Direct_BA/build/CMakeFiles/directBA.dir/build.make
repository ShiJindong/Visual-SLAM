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
CMAKE_SOURCE_DIR = /home/jindong/SLAM/Chap7/L7_code/Direct_BA

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jindong/SLAM/Chap7/L7_code/Direct_BA/build

# Include any dependencies generated for this target.
include CMakeFiles/directBA.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/directBA.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/directBA.dir/flags.make

CMakeFiles/directBA.dir/directBA.cpp.o: CMakeFiles/directBA.dir/flags.make
CMakeFiles/directBA.dir/directBA.cpp.o: ../directBA.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jindong/SLAM/Chap7/L7_code/Direct_BA/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/directBA.dir/directBA.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/directBA.dir/directBA.cpp.o -c /home/jindong/SLAM/Chap7/L7_code/Direct_BA/directBA.cpp

CMakeFiles/directBA.dir/directBA.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/directBA.dir/directBA.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jindong/SLAM/Chap7/L7_code/Direct_BA/directBA.cpp > CMakeFiles/directBA.dir/directBA.cpp.i

CMakeFiles/directBA.dir/directBA.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/directBA.dir/directBA.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jindong/SLAM/Chap7/L7_code/Direct_BA/directBA.cpp -o CMakeFiles/directBA.dir/directBA.cpp.s

# Object files for target directBA
directBA_OBJECTS = \
"CMakeFiles/directBA.dir/directBA.cpp.o"

# External object files for target directBA
directBA_EXTERNAL_OBJECTS =

../OUTPUT/directBA: CMakeFiles/directBA.dir/directBA.cpp.o
../OUTPUT/directBA: CMakeFiles/directBA.dir/build.make
../OUTPUT/directBA: /usr/lib/x86_64-linux-gnu/libcholmod.so
../OUTPUT/directBA: /usr/lib/x86_64-linux-gnu/libamd.so
../OUTPUT/directBA: /usr/lib/x86_64-linux-gnu/libcolamd.so
../OUTPUT/directBA: /usr/lib/x86_64-linux-gnu/libcamd.so
../OUTPUT/directBA: /usr/lib/x86_64-linux-gnu/libccolamd.so
../OUTPUT/directBA: /usr/lib/x86_64-linux-gnu/libsuitesparseconfig.so
../OUTPUT/directBA: /usr/local/lib/libSophus.so
../OUTPUT/directBA: /usr/local/lib/libopencv_dnn.so.3.4.15
../OUTPUT/directBA: /usr/local/lib/libopencv_highgui.so.3.4.15
../OUTPUT/directBA: /usr/local/lib/libopencv_ml.so.3.4.15
../OUTPUT/directBA: /usr/local/lib/libopencv_objdetect.so.3.4.15
../OUTPUT/directBA: /usr/local/lib/libopencv_shape.so.3.4.15
../OUTPUT/directBA: /usr/local/lib/libopencv_stitching.so.3.4.15
../OUTPUT/directBA: /usr/local/lib/libopencv_superres.so.3.4.15
../OUTPUT/directBA: /usr/local/lib/libopencv_videostab.so.3.4.15
../OUTPUT/directBA: /usr/local/lib/libopencv_viz.so.3.4.15
../OUTPUT/directBA: /usr/local/lib/libpango_glgeometry.so
../OUTPUT/directBA: /usr/local/lib/libpango_plot.so
../OUTPUT/directBA: /usr/local/lib/libpango_python.so
../OUTPUT/directBA: /usr/local/lib/libpango_scene.so
../OUTPUT/directBA: /usr/local/lib/libpango_tools.so
../OUTPUT/directBA: /usr/local/lib/libpango_video.so
../OUTPUT/directBA: /usr/local/lib/libopencv_calib3d.so.3.4.15
../OUTPUT/directBA: /usr/local/lib/libopencv_features2d.so.3.4.15
../OUTPUT/directBA: /usr/local/lib/libopencv_flann.so.3.4.15
../OUTPUT/directBA: /usr/local/lib/libopencv_photo.so.3.4.15
../OUTPUT/directBA: /usr/local/lib/libopencv_video.so.3.4.15
../OUTPUT/directBA: /usr/local/lib/libopencv_videoio.so.3.4.15
../OUTPUT/directBA: /usr/local/lib/libopencv_imgcodecs.so.3.4.15
../OUTPUT/directBA: /usr/local/lib/libopencv_imgproc.so.3.4.15
../OUTPUT/directBA: /usr/local/lib/libopencv_core.so.3.4.15
../OUTPUT/directBA: /usr/local/lib/libpango_geometry.so
../OUTPUT/directBA: /usr/local/lib/libtinyobj.so
../OUTPUT/directBA: /usr/local/lib/libpango_display.so
../OUTPUT/directBA: /usr/local/lib/libpango_vars.so
../OUTPUT/directBA: /usr/local/lib/libpango_windowing.so
../OUTPUT/directBA: /usr/local/lib/libpango_opengl.so
../OUTPUT/directBA: /usr/lib/x86_64-linux-gnu/libGLEW.so
../OUTPUT/directBA: /usr/lib/x86_64-linux-gnu/libOpenGL.so
../OUTPUT/directBA: /usr/lib/x86_64-linux-gnu/libGLX.so
../OUTPUT/directBA: /usr/lib/x86_64-linux-gnu/libGLU.so
../OUTPUT/directBA: /usr/local/lib/libpango_image.so
../OUTPUT/directBA: /usr/local/lib/libpango_packetstream.so
../OUTPUT/directBA: /usr/local/lib/libpango_core.so
../OUTPUT/directBA: CMakeFiles/directBA.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jindong/SLAM/Chap7/L7_code/Direct_BA/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../OUTPUT/directBA"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/directBA.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/directBA.dir/build: ../OUTPUT/directBA

.PHONY : CMakeFiles/directBA.dir/build

CMakeFiles/directBA.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/directBA.dir/cmake_clean.cmake
.PHONY : CMakeFiles/directBA.dir/clean

CMakeFiles/directBA.dir/depend:
	cd /home/jindong/SLAM/Chap7/L7_code/Direct_BA/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jindong/SLAM/Chap7/L7_code/Direct_BA /home/jindong/SLAM/Chap7/L7_code/Direct_BA /home/jindong/SLAM/Chap7/L7_code/Direct_BA/build /home/jindong/SLAM/Chap7/L7_code/Direct_BA/build /home/jindong/SLAM/Chap7/L7_code/Direct_BA/build/CMakeFiles/directBA.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/directBA.dir/depend

