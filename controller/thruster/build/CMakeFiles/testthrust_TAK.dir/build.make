# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/skloe/catkin_ws4/src/dp_core/src/testosqp

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/skloe/catkin_ws4/src/dp_core/src/testosqp/build

# Include any dependencies generated for this target.
include CMakeFiles/testthrust_TAK.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/testthrust_TAK.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/testthrust_TAK.dir/flags.make

CMakeFiles/testthrust_TAK.dir/testthrust_TAK.cc.o: CMakeFiles/testthrust_TAK.dir/flags.make
CMakeFiles/testthrust_TAK.dir/testthrust_TAK.cc.o: ../testthrust_TAK.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/skloe/catkin_ws4/src/dp_core/src/testosqp/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/testthrust_TAK.dir/testthrust_TAK.cc.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/testthrust_TAK.dir/testthrust_TAK.cc.o -c /home/skloe/catkin_ws4/src/dp_core/src/testosqp/testthrust_TAK.cc

CMakeFiles/testthrust_TAK.dir/testthrust_TAK.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/testthrust_TAK.dir/testthrust_TAK.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/skloe/catkin_ws4/src/dp_core/src/testosqp/testthrust_TAK.cc > CMakeFiles/testthrust_TAK.dir/testthrust_TAK.cc.i

CMakeFiles/testthrust_TAK.dir/testthrust_TAK.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/testthrust_TAK.dir/testthrust_TAK.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/skloe/catkin_ws4/src/dp_core/src/testosqp/testthrust_TAK.cc -o CMakeFiles/testthrust_TAK.dir/testthrust_TAK.cc.s

CMakeFiles/testthrust_TAK.dir/testthrust_TAK.cc.o.requires:

.PHONY : CMakeFiles/testthrust_TAK.dir/testthrust_TAK.cc.o.requires

CMakeFiles/testthrust_TAK.dir/testthrust_TAK.cc.o.provides: CMakeFiles/testthrust_TAK.dir/testthrust_TAK.cc.o.requires
	$(MAKE) -f CMakeFiles/testthrust_TAK.dir/build.make CMakeFiles/testthrust_TAK.dir/testthrust_TAK.cc.o.provides.build
.PHONY : CMakeFiles/testthrust_TAK.dir/testthrust_TAK.cc.o.provides

CMakeFiles/testthrust_TAK.dir/testthrust_TAK.cc.o.provides.build: CMakeFiles/testthrust_TAK.dir/testthrust_TAK.cc.o


# Object files for target testthrust_TAK
testthrust_TAK_OBJECTS = \
"CMakeFiles/testthrust_TAK.dir/testthrust_TAK.cc.o"

# External object files for target testthrust_TAK
testthrust_TAK_EXTERNAL_OBJECTS =

testthrust_TAK: CMakeFiles/testthrust_TAK.dir/testthrust_TAK.cc.o
testthrust_TAK: CMakeFiles/testthrust_TAK.dir/build.make
testthrust_TAK: /home/skloe/catkin_ws4/src/dp_core/third_party/osqp/build/out/libosqp.so
testthrust_TAK: CMakeFiles/testthrust_TAK.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/skloe/catkin_ws4/src/dp_core/src/testosqp/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable testthrust_TAK"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/testthrust_TAK.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/testthrust_TAK.dir/build: testthrust_TAK

.PHONY : CMakeFiles/testthrust_TAK.dir/build

CMakeFiles/testthrust_TAK.dir/requires: CMakeFiles/testthrust_TAK.dir/testthrust_TAK.cc.o.requires

.PHONY : CMakeFiles/testthrust_TAK.dir/requires

CMakeFiles/testthrust_TAK.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/testthrust_TAK.dir/cmake_clean.cmake
.PHONY : CMakeFiles/testthrust_TAK.dir/clean

CMakeFiles/testthrust_TAK.dir/depend:
	cd /home/skloe/catkin_ws4/src/dp_core/src/testosqp/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/skloe/catkin_ws4/src/dp_core/src/testosqp /home/skloe/catkin_ws4/src/dp_core/src/testosqp /home/skloe/catkin_ws4/src/dp_core/src/testosqp/build /home/skloe/catkin_ws4/src/dp_core/src/testosqp/build /home/skloe/catkin_ws4/src/dp_core/src/testosqp/build/CMakeFiles/testthrust_TAK.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/testthrust_TAK.dir/depend

