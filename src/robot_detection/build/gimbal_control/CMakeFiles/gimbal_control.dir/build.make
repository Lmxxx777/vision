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
CMAKE_SOURCE_DIR = /home/lmx2/robot_detection

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/lmx2/robot_detection/build

# Include any dependencies generated for this target.
include gimbal_control/CMakeFiles/gimbal_control.dir/depend.make

# Include the progress variables for this target.
include gimbal_control/CMakeFiles/gimbal_control.dir/progress.make

# Include the compile flags for this target's objects.
include gimbal_control/CMakeFiles/gimbal_control.dir/flags.make

gimbal_control/CMakeFiles/gimbal_control.dir/gimbal_control.cpp.o: gimbal_control/CMakeFiles/gimbal_control.dir/flags.make
gimbal_control/CMakeFiles/gimbal_control.dir/gimbal_control.cpp.o: ../gimbal_control/gimbal_control.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lmx2/robot_detection/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object gimbal_control/CMakeFiles/gimbal_control.dir/gimbal_control.cpp.o"
	cd /home/lmx2/robot_detection/build/gimbal_control && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/gimbal_control.dir/gimbal_control.cpp.o -c /home/lmx2/robot_detection/gimbal_control/gimbal_control.cpp

gimbal_control/CMakeFiles/gimbal_control.dir/gimbal_control.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gimbal_control.dir/gimbal_control.cpp.i"
	cd /home/lmx2/robot_detection/build/gimbal_control && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lmx2/robot_detection/gimbal_control/gimbal_control.cpp > CMakeFiles/gimbal_control.dir/gimbal_control.cpp.i

gimbal_control/CMakeFiles/gimbal_control.dir/gimbal_control.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gimbal_control.dir/gimbal_control.cpp.s"
	cd /home/lmx2/robot_detection/build/gimbal_control && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lmx2/robot_detection/gimbal_control/gimbal_control.cpp -o CMakeFiles/gimbal_control.dir/gimbal_control.cpp.s

# Object files for target gimbal_control
gimbal_control_OBJECTS = \
"CMakeFiles/gimbal_control.dir/gimbal_control.cpp.o"

# External object files for target gimbal_control
gimbal_control_EXTERNAL_OBJECTS =

gimbal_control/libgimbal_control.a: gimbal_control/CMakeFiles/gimbal_control.dir/gimbal_control.cpp.o
gimbal_control/libgimbal_control.a: gimbal_control/CMakeFiles/gimbal_control.dir/build.make
gimbal_control/libgimbal_control.a: gimbal_control/CMakeFiles/gimbal_control.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/lmx2/robot_detection/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libgimbal_control.a"
	cd /home/lmx2/robot_detection/build/gimbal_control && $(CMAKE_COMMAND) -P CMakeFiles/gimbal_control.dir/cmake_clean_target.cmake
	cd /home/lmx2/robot_detection/build/gimbal_control && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/gimbal_control.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
gimbal_control/CMakeFiles/gimbal_control.dir/build: gimbal_control/libgimbal_control.a

.PHONY : gimbal_control/CMakeFiles/gimbal_control.dir/build

gimbal_control/CMakeFiles/gimbal_control.dir/clean:
	cd /home/lmx2/robot_detection/build/gimbal_control && $(CMAKE_COMMAND) -P CMakeFiles/gimbal_control.dir/cmake_clean.cmake
.PHONY : gimbal_control/CMakeFiles/gimbal_control.dir/clean

gimbal_control/CMakeFiles/gimbal_control.dir/depend:
	cd /home/lmx2/robot_detection/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/lmx2/robot_detection /home/lmx2/robot_detection/gimbal_control /home/lmx2/robot_detection/build /home/lmx2/robot_detection/build/gimbal_control /home/lmx2/robot_detection/build/gimbal_control/CMakeFiles/gimbal_control.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : gimbal_control/CMakeFiles/gimbal_control.dir/depend

