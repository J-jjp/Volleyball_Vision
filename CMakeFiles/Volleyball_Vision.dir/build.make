# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

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

#Suppress display of executed commands.
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
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/jjp/Volleyball

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jjp/Volleyball/volleyball

# Include any dependencies generated for this target.
include CMakeFiles/Volleyball_Vision.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/Volleyball_Vision.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/Volleyball_Vision.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Volleyball_Vision.dir/flags.make

CMakeFiles/Volleyball_Vision.dir/Volleyball_Vision.cpp.o: CMakeFiles/Volleyball_Vision.dir/flags.make
CMakeFiles/Volleyball_Vision.dir/Volleyball_Vision.cpp.o: ../Volleyball_Vision.cpp
CMakeFiles/Volleyball_Vision.dir/Volleyball_Vision.cpp.o: CMakeFiles/Volleyball_Vision.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jjp/Volleyball/volleyball/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/Volleyball_Vision.dir/Volleyball_Vision.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/Volleyball_Vision.dir/Volleyball_Vision.cpp.o -MF CMakeFiles/Volleyball_Vision.dir/Volleyball_Vision.cpp.o.d -o CMakeFiles/Volleyball_Vision.dir/Volleyball_Vision.cpp.o -c /home/jjp/Volleyball/Volleyball_Vision.cpp

CMakeFiles/Volleyball_Vision.dir/Volleyball_Vision.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Volleyball_Vision.dir/Volleyball_Vision.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jjp/Volleyball/Volleyball_Vision.cpp > CMakeFiles/Volleyball_Vision.dir/Volleyball_Vision.cpp.i

CMakeFiles/Volleyball_Vision.dir/Volleyball_Vision.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Volleyball_Vision.dir/Volleyball_Vision.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jjp/Volleyball/Volleyball_Vision.cpp -o CMakeFiles/Volleyball_Vision.dir/Volleyball_Vision.cpp.s

# Object files for target Volleyball_Vision
Volleyball_Vision_OBJECTS = \
"CMakeFiles/Volleyball_Vision.dir/Volleyball_Vision.cpp.o"

# External object files for target Volleyball_Vision
Volleyball_Vision_EXTERNAL_OBJECTS =

Volleyball_Vision: CMakeFiles/Volleyball_Vision.dir/Volleyball_Vision.cpp.o
Volleyball_Vision: CMakeFiles/Volleyball_Vision.dir/build.make
Volleyball_Vision: /usr/local/lib/librealsense2.so.2.54.2
Volleyball_Vision: /usr/local/lib/libopencv_gapi.so.4.8.0
Volleyball_Vision: /usr/local/lib/libopencv_highgui.so.4.8.0
Volleyball_Vision: /usr/local/lib/libopencv_ml.so.4.8.0
Volleyball_Vision: /usr/local/lib/libopencv_objdetect.so.4.8.0
Volleyball_Vision: /usr/local/lib/libopencv_photo.so.4.8.0
Volleyball_Vision: /usr/local/lib/libopencv_stitching.so.4.8.0
Volleyball_Vision: /usr/local/lib/libopencv_video.so.4.8.0
Volleyball_Vision: /usr/local/lib/libopencv_videoio.so.4.8.0
Volleyball_Vision: /usr/local/lib/librsutils.a
Volleyball_Vision: /usr/local/lib/libopencv_imgcodecs.so.4.8.0
Volleyball_Vision: /usr/local/lib/libopencv_dnn.so.4.8.0
Volleyball_Vision: /usr/local/lib/libopencv_calib3d.so.4.8.0
Volleyball_Vision: /usr/local/lib/libopencv_features2d.so.4.8.0
Volleyball_Vision: /usr/local/lib/libopencv_flann.so.4.8.0
Volleyball_Vision: /usr/local/lib/libopencv_imgproc.so.4.8.0
Volleyball_Vision: /usr/local/lib/libopencv_core.so.4.8.0
Volleyball_Vision: CMakeFiles/Volleyball_Vision.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jjp/Volleyball/volleyball/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable Volleyball_Vision"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Volleyball_Vision.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Volleyball_Vision.dir/build: Volleyball_Vision
.PHONY : CMakeFiles/Volleyball_Vision.dir/build

CMakeFiles/Volleyball_Vision.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Volleyball_Vision.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Volleyball_Vision.dir/clean

CMakeFiles/Volleyball_Vision.dir/depend:
	cd /home/jjp/Volleyball/volleyball && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jjp/Volleyball /home/jjp/Volleyball /home/jjp/Volleyball/volleyball /home/jjp/Volleyball/volleyball /home/jjp/Volleyball/volleyball/CMakeFiles/Volleyball_Vision.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Volleyball_Vision.dir/depend

