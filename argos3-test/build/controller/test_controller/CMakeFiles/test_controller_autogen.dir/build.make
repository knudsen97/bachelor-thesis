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
CMAKE_SOURCE_DIR = /home/kristian/Documents/sdu/bachelor/argos3-test

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/kristian/Documents/sdu/bachelor/argos3-test/build

# Utility rule file for test_controller_autogen.

# Include the progress variables for this target.
include controller/test_controller/CMakeFiles/test_controller_autogen.dir/progress.make

controller/test_controller/CMakeFiles/test_controller_autogen:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/kristian/Documents/sdu/bachelor/argos3-test/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Automatic MOC for target test_controller"
	cd /home/kristian/Documents/sdu/bachelor/argos3-test/build/controller/test_controller && /usr/bin/cmake -E cmake_autogen /home/kristian/Documents/sdu/bachelor/argos3-test/build/controller/test_controller/CMakeFiles/test_controller_autogen.dir/AutogenInfo.json Debug

test_controller_autogen: controller/test_controller/CMakeFiles/test_controller_autogen
test_controller_autogen: controller/test_controller/CMakeFiles/test_controller_autogen.dir/build.make

.PHONY : test_controller_autogen

# Rule to build all files generated by this target.
controller/test_controller/CMakeFiles/test_controller_autogen.dir/build: test_controller_autogen

.PHONY : controller/test_controller/CMakeFiles/test_controller_autogen.dir/build

controller/test_controller/CMakeFiles/test_controller_autogen.dir/clean:
	cd /home/kristian/Documents/sdu/bachelor/argos3-test/build/controller/test_controller && $(CMAKE_COMMAND) -P CMakeFiles/test_controller_autogen.dir/cmake_clean.cmake
.PHONY : controller/test_controller/CMakeFiles/test_controller_autogen.dir/clean

controller/test_controller/CMakeFiles/test_controller_autogen.dir/depend:
	cd /home/kristian/Documents/sdu/bachelor/argos3-test/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kristian/Documents/sdu/bachelor/argos3-test /home/kristian/Documents/sdu/bachelor/argos3-test/controller/test_controller /home/kristian/Documents/sdu/bachelor/argos3-test/build /home/kristian/Documents/sdu/bachelor/argos3-test/build/controller/test_controller /home/kristian/Documents/sdu/bachelor/argos3-test/build/controller/test_controller/CMakeFiles/test_controller_autogen.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : controller/test_controller/CMakeFiles/test_controller_autogen.dir/depend

