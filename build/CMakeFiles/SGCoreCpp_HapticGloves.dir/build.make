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
CMAKE_SOURCE_DIR = /home/harlab/ros_ws_1/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/harlab/ros_ws_1/build

# Include any dependencies generated for this target.
include CMakeFiles/SGCoreCpp_HapticGloves.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/SGCoreCpp_HapticGloves.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/SGCoreCpp_HapticGloves.dir/flags.make

CMakeFiles/SGCoreCpp_HapticGloves.dir/sense_glove_test/src/SGCoreCpp_HapticGloves.cpp.o: CMakeFiles/SGCoreCpp_HapticGloves.dir/flags.make
CMakeFiles/SGCoreCpp_HapticGloves.dir/sense_glove_test/src/SGCoreCpp_HapticGloves.cpp.o: /home/harlab/ros_ws_1/src/sense_glove_test/src/SGCoreCpp_HapticGloves.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/harlab/ros_ws_1/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/SGCoreCpp_HapticGloves.dir/sense_glove_test/src/SGCoreCpp_HapticGloves.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/SGCoreCpp_HapticGloves.dir/sense_glove_test/src/SGCoreCpp_HapticGloves.cpp.o -c /home/harlab/ros_ws_1/src/sense_glove_test/src/SGCoreCpp_HapticGloves.cpp

CMakeFiles/SGCoreCpp_HapticGloves.dir/sense_glove_test/src/SGCoreCpp_HapticGloves.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/SGCoreCpp_HapticGloves.dir/sense_glove_test/src/SGCoreCpp_HapticGloves.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/harlab/ros_ws_1/src/sense_glove_test/src/SGCoreCpp_HapticGloves.cpp > CMakeFiles/SGCoreCpp_HapticGloves.dir/sense_glove_test/src/SGCoreCpp_HapticGloves.cpp.i

CMakeFiles/SGCoreCpp_HapticGloves.dir/sense_glove_test/src/SGCoreCpp_HapticGloves.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/SGCoreCpp_HapticGloves.dir/sense_glove_test/src/SGCoreCpp_HapticGloves.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/harlab/ros_ws_1/src/sense_glove_test/src/SGCoreCpp_HapticGloves.cpp -o CMakeFiles/SGCoreCpp_HapticGloves.dir/sense_glove_test/src/SGCoreCpp_HapticGloves.cpp.s

# Object files for target SGCoreCpp_HapticGloves
SGCoreCpp_HapticGloves_OBJECTS = \
"CMakeFiles/SGCoreCpp_HapticGloves.dir/sense_glove_test/src/SGCoreCpp_HapticGloves.cpp.o"

# External object files for target SGCoreCpp_HapticGloves
SGCoreCpp_HapticGloves_EXTERNAL_OBJECTS =

SGCoreCpp_HapticGloves: CMakeFiles/SGCoreCpp_HapticGloves.dir/sense_glove_test/src/SGCoreCpp_HapticGloves.cpp.o
SGCoreCpp_HapticGloves: CMakeFiles/SGCoreCpp_HapticGloves.dir/build.make
SGCoreCpp_HapticGloves: /home/harlab/ros_ws_1/src/sense_glove_test/include/libSGCoreCpp.so
SGCoreCpp_HapticGloves: CMakeFiles/SGCoreCpp_HapticGloves.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/harlab/ros_ws_1/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable SGCoreCpp_HapticGloves"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/SGCoreCpp_HapticGloves.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/SGCoreCpp_HapticGloves.dir/build: SGCoreCpp_HapticGloves

.PHONY : CMakeFiles/SGCoreCpp_HapticGloves.dir/build

CMakeFiles/SGCoreCpp_HapticGloves.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/SGCoreCpp_HapticGloves.dir/cmake_clean.cmake
.PHONY : CMakeFiles/SGCoreCpp_HapticGloves.dir/clean

CMakeFiles/SGCoreCpp_HapticGloves.dir/depend:
	cd /home/harlab/ros_ws_1/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/harlab/ros_ws_1/src /home/harlab/ros_ws_1/src /home/harlab/ros_ws_1/build /home/harlab/ros_ws_1/build /home/harlab/ros_ws_1/build/CMakeFiles/SGCoreCpp_HapticGloves.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/SGCoreCpp_HapticGloves.dir/depend

