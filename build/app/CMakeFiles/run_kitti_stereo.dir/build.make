# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /mnt/c/Users/JLK/Desktop/Robo/slamdir/slambook2/ch13

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /mnt/c/Users/JLK/Desktop/Robo/slamdir/slambook2/ch13/build

# Include any dependencies generated for this target.
include app/CMakeFiles/run_kitti_stereo.dir/depend.make

# Include the progress variables for this target.
include app/CMakeFiles/run_kitti_stereo.dir/progress.make

# Include the compile flags for this target's objects.
include app/CMakeFiles/run_kitti_stereo.dir/flags.make

app/CMakeFiles/run_kitti_stereo.dir/run_kitti_stereo.cpp.o: app/CMakeFiles/run_kitti_stereo.dir/flags.make
app/CMakeFiles/run_kitti_stereo.dir/run_kitti_stereo.cpp.o: ../app/run_kitti_stereo.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/mnt/c/Users/JLK/Desktop/Robo/slamdir/slambook2/ch13/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object app/CMakeFiles/run_kitti_stereo.dir/run_kitti_stereo.cpp.o"
	cd /mnt/c/Users/JLK/Desktop/Robo/slamdir/slambook2/ch13/build/app && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/run_kitti_stereo.dir/run_kitti_stereo.cpp.o -c /mnt/c/Users/JLK/Desktop/Robo/slamdir/slambook2/ch13/app/run_kitti_stereo.cpp

app/CMakeFiles/run_kitti_stereo.dir/run_kitti_stereo.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/run_kitti_stereo.dir/run_kitti_stereo.cpp.i"
	cd /mnt/c/Users/JLK/Desktop/Robo/slamdir/slambook2/ch13/build/app && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /mnt/c/Users/JLK/Desktop/Robo/slamdir/slambook2/ch13/app/run_kitti_stereo.cpp > CMakeFiles/run_kitti_stereo.dir/run_kitti_stereo.cpp.i

app/CMakeFiles/run_kitti_stereo.dir/run_kitti_stereo.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/run_kitti_stereo.dir/run_kitti_stereo.cpp.s"
	cd /mnt/c/Users/JLK/Desktop/Robo/slamdir/slambook2/ch13/build/app && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /mnt/c/Users/JLK/Desktop/Robo/slamdir/slambook2/ch13/app/run_kitti_stereo.cpp -o CMakeFiles/run_kitti_stereo.dir/run_kitti_stereo.cpp.s

# Object files for target run_kitti_stereo
run_kitti_stereo_OBJECTS = \
"CMakeFiles/run_kitti_stereo.dir/run_kitti_stereo.cpp.o"

# External object files for target run_kitti_stereo
run_kitti_stereo_EXTERNAL_OBJECTS =

../bin/run_kitti_stereo: app/CMakeFiles/run_kitti_stereo.dir/run_kitti_stereo.cpp.o
../bin/run_kitti_stereo: app/CMakeFiles/run_kitti_stereo.dir/build.make
../bin/run_kitti_stereo: /usr/local/lib/libfmt.a
../bin/run_kitti_stereo: ../lib/libmyslam.so
../bin/run_kitti_stereo: /usr/local/lib/libopencv_dnn.so.4.5.0
../bin/run_kitti_stereo: /usr/local/lib/libopencv_gapi.so.4.5.0
../bin/run_kitti_stereo: /usr/local/lib/libopencv_highgui.so.4.5.0
../bin/run_kitti_stereo: /usr/local/lib/libopencv_ml.so.4.5.0
../bin/run_kitti_stereo: /usr/local/lib/libopencv_objdetect.so.4.5.0
../bin/run_kitti_stereo: /usr/local/lib/libopencv_photo.so.4.5.0
../bin/run_kitti_stereo: /usr/local/lib/libopencv_stitching.so.4.5.0
../bin/run_kitti_stereo: /usr/local/lib/libopencv_video.so.4.5.0
../bin/run_kitti_stereo: /usr/local/lib/libopencv_videoio.so.4.5.0
../bin/run_kitti_stereo: /mnt/c/Users/JLK/Desktop/Robo/thridparty/build/libpango_glgeometry.so
../bin/run_kitti_stereo: /mnt/c/Users/JLK/Desktop/Robo/thridparty/build/libpango_plot.so
../bin/run_kitti_stereo: /mnt/c/Users/JLK/Desktop/Robo/thridparty/build/libpango_python.so
../bin/run_kitti_stereo: /mnt/c/Users/JLK/Desktop/Robo/thridparty/build/libpango_scene.so
../bin/run_kitti_stereo: /mnt/c/Users/JLK/Desktop/Robo/thridparty/build/libpango_tools.so
../bin/run_kitti_stereo: /mnt/c/Users/JLK/Desktop/Robo/thridparty/build/libpango_video.so
../bin/run_kitti_stereo: /usr/local/lib/libgtest.a
../bin/run_kitti_stereo: /usr/local/lib/libgtest_main.a
../bin/run_kitti_stereo: /usr/lib/x86_64-linux-gnu/libglog.so
../bin/run_kitti_stereo: /usr/lib/x86_64-linux-gnu/libgflags.so.2.2.2
../bin/run_kitti_stereo: /usr/lib/x86_64-linux-gnu/libcxsparse.so
../bin/run_kitti_stereo: /usr/local/lib/libfmt.a
../bin/run_kitti_stereo: /usr/local/lib/libopencv_calib3d.so.4.5.0
../bin/run_kitti_stereo: /usr/local/lib/libopencv_features2d.so.4.5.0
../bin/run_kitti_stereo: /usr/local/lib/libopencv_flann.so.4.5.0
../bin/run_kitti_stereo: /usr/local/lib/libopencv_imgcodecs.so.4.5.0
../bin/run_kitti_stereo: /usr/local/lib/libopencv_imgproc.so.4.5.0
../bin/run_kitti_stereo: /usr/local/lib/libopencv_core.so.4.5.0
../bin/run_kitti_stereo: /mnt/c/Users/JLK/Desktop/Robo/thridparty/build/libpango_geometry.so
../bin/run_kitti_stereo: /mnt/c/Users/JLK/Desktop/Robo/thridparty/build/libpango_display.so
../bin/run_kitti_stereo: /mnt/c/Users/JLK/Desktop/Robo/thridparty/build/libpango_vars.so
../bin/run_kitti_stereo: /mnt/c/Users/JLK/Desktop/Robo/thridparty/build/libpango_packetstream.so
../bin/run_kitti_stereo: /mnt/c/Users/JLK/Desktop/Robo/thridparty/build/libpango_windowing.so
../bin/run_kitti_stereo: /mnt/c/Users/JLK/Desktop/Robo/thridparty/build/libpango_opengl.so
../bin/run_kitti_stereo: /mnt/c/Users/JLK/Desktop/Robo/thridparty/build/libpango_image.so
../bin/run_kitti_stereo: /mnt/c/Users/JLK/Desktop/Robo/thridparty/build/libpango_core.so
../bin/run_kitti_stereo: /usr/lib/x86_64-linux-gnu/libGLEW.so
../bin/run_kitti_stereo: /usr/lib/x86_64-linux-gnu/libOpenGL.so
../bin/run_kitti_stereo: /usr/lib/x86_64-linux-gnu/libGLX.so
../bin/run_kitti_stereo: /usr/lib/x86_64-linux-gnu/libGLU.so
../bin/run_kitti_stereo: /mnt/c/Users/JLK/Desktop/Robo/thridparty/build/libtinyobj.so
../bin/run_kitti_stereo: /usr/local/lib/libgtest.a
../bin/run_kitti_stereo: app/CMakeFiles/run_kitti_stereo.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/mnt/c/Users/JLK/Desktop/Robo/slamdir/slambook2/ch13/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../bin/run_kitti_stereo"
	cd /mnt/c/Users/JLK/Desktop/Robo/slamdir/slambook2/ch13/build/app && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/run_kitti_stereo.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
app/CMakeFiles/run_kitti_stereo.dir/build: ../bin/run_kitti_stereo

.PHONY : app/CMakeFiles/run_kitti_stereo.dir/build

app/CMakeFiles/run_kitti_stereo.dir/clean:
	cd /mnt/c/Users/JLK/Desktop/Robo/slamdir/slambook2/ch13/build/app && $(CMAKE_COMMAND) -P CMakeFiles/run_kitti_stereo.dir/cmake_clean.cmake
.PHONY : app/CMakeFiles/run_kitti_stereo.dir/clean

app/CMakeFiles/run_kitti_stereo.dir/depend:
	cd /mnt/c/Users/JLK/Desktop/Robo/slamdir/slambook2/ch13/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /mnt/c/Users/JLK/Desktop/Robo/slamdir/slambook2/ch13 /mnt/c/Users/JLK/Desktop/Robo/slamdir/slambook2/ch13/app /mnt/c/Users/JLK/Desktop/Robo/slamdir/slambook2/ch13/build /mnt/c/Users/JLK/Desktop/Robo/slamdir/slambook2/ch13/build/app /mnt/c/Users/JLK/Desktop/Robo/slamdir/slambook2/ch13/build/app/CMakeFiles/run_kitti_stereo.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : app/CMakeFiles/run_kitti_stereo.dir/depend

