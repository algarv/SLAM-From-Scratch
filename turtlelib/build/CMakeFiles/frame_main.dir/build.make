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
CMAKE_SOURCE_DIR = /home/algarverick/ME_495_SLAM/src/slam-project-algarv/turtlelib

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/algarverick/ME_495_SLAM/src/slam-project-algarv/turtlelib/build

# Include any dependencies generated for this target.
include CMakeFiles/frame_main.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/frame_main.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/frame_main.dir/flags.make

CMakeFiles/frame_main.dir/src/frame_main.cpp.o: CMakeFiles/frame_main.dir/flags.make
CMakeFiles/frame_main.dir/src/frame_main.cpp.o: ../src/frame_main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/algarverick/ME_495_SLAM/src/slam-project-algarv/turtlelib/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/frame_main.dir/src/frame_main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/frame_main.dir/src/frame_main.cpp.o -c /home/algarverick/ME_495_SLAM/src/slam-project-algarv/turtlelib/src/frame_main.cpp

CMakeFiles/frame_main.dir/src/frame_main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/frame_main.dir/src/frame_main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/algarverick/ME_495_SLAM/src/slam-project-algarv/turtlelib/src/frame_main.cpp > CMakeFiles/frame_main.dir/src/frame_main.cpp.i

CMakeFiles/frame_main.dir/src/frame_main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/frame_main.dir/src/frame_main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/algarverick/ME_495_SLAM/src/slam-project-algarv/turtlelib/src/frame_main.cpp -o CMakeFiles/frame_main.dir/src/frame_main.cpp.s

# Object files for target frame_main
frame_main_OBJECTS = \
"CMakeFiles/frame_main.dir/src/frame_main.cpp.o"

# External object files for target frame_main
frame_main_EXTERNAL_OBJECTS =

frame_main: CMakeFiles/frame_main.dir/src/frame_main.cpp.o
frame_main: CMakeFiles/frame_main.dir/build.make
frame_main: libturtlelib.a
frame_main: CMakeFiles/frame_main.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/algarverick/ME_495_SLAM/src/slam-project-algarv/turtlelib/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable frame_main"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/frame_main.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/frame_main.dir/build: frame_main

.PHONY : CMakeFiles/frame_main.dir/build

CMakeFiles/frame_main.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/frame_main.dir/cmake_clean.cmake
.PHONY : CMakeFiles/frame_main.dir/clean

CMakeFiles/frame_main.dir/depend:
	cd /home/algarverick/ME_495_SLAM/src/slam-project-algarv/turtlelib/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/algarverick/ME_495_SLAM/src/slam-project-algarv/turtlelib /home/algarverick/ME_495_SLAM/src/slam-project-algarv/turtlelib /home/algarverick/ME_495_SLAM/src/slam-project-algarv/turtlelib/build /home/algarverick/ME_495_SLAM/src/slam-project-algarv/turtlelib/build /home/algarverick/ME_495_SLAM/src/slam-project-algarv/turtlelib/build/CMakeFiles/frame_main.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/frame_main.dir/depend

