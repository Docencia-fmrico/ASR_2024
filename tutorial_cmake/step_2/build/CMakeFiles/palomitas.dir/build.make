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
CMAKE_SOURCE_DIR = /home/fmrico/ros/ros2/asr24_ws/src/ASR_2024/tutorial_cmake/step_2

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/fmrico/ros/ros2/asr24_ws/src/ASR_2024/tutorial_cmake/step_2/build

# Include any dependencies generated for this target.
include CMakeFiles/palomitas.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/palomitas.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/palomitas.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/palomitas.dir/flags.make

CMakeFiles/palomitas.dir/src/cmake_tutorial_2/functions.cpp.o: CMakeFiles/palomitas.dir/flags.make
CMakeFiles/palomitas.dir/src/cmake_tutorial_2/functions.cpp.o: ../src/cmake_tutorial_2/functions.cpp
CMakeFiles/palomitas.dir/src/cmake_tutorial_2/functions.cpp.o: CMakeFiles/palomitas.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/fmrico/ros/ros2/asr24_ws/src/ASR_2024/tutorial_cmake/step_2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/palomitas.dir/src/cmake_tutorial_2/functions.cpp.o"
	/usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/palomitas.dir/src/cmake_tutorial_2/functions.cpp.o -MF CMakeFiles/palomitas.dir/src/cmake_tutorial_2/functions.cpp.o.d -o CMakeFiles/palomitas.dir/src/cmake_tutorial_2/functions.cpp.o -c /home/fmrico/ros/ros2/asr24_ws/src/ASR_2024/tutorial_cmake/step_2/src/cmake_tutorial_2/functions.cpp

CMakeFiles/palomitas.dir/src/cmake_tutorial_2/functions.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/palomitas.dir/src/cmake_tutorial_2/functions.cpp.i"
	/usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/fmrico/ros/ros2/asr24_ws/src/ASR_2024/tutorial_cmake/step_2/src/cmake_tutorial_2/functions.cpp > CMakeFiles/palomitas.dir/src/cmake_tutorial_2/functions.cpp.i

CMakeFiles/palomitas.dir/src/cmake_tutorial_2/functions.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/palomitas.dir/src/cmake_tutorial_2/functions.cpp.s"
	/usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/fmrico/ros/ros2/asr24_ws/src/ASR_2024/tutorial_cmake/step_2/src/cmake_tutorial_2/functions.cpp -o CMakeFiles/palomitas.dir/src/cmake_tutorial_2/functions.cpp.s

# Object files for target palomitas
palomitas_OBJECTS = \
"CMakeFiles/palomitas.dir/src/cmake_tutorial_2/functions.cpp.o"

# External object files for target palomitas
palomitas_EXTERNAL_OBJECTS =

libpalomitas.so: CMakeFiles/palomitas.dir/src/cmake_tutorial_2/functions.cpp.o
libpalomitas.so: CMakeFiles/palomitas.dir/build.make
libpalomitas.so: CMakeFiles/palomitas.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/fmrico/ros/ros2/asr24_ws/src/ASR_2024/tutorial_cmake/step_2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library libpalomitas.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/palomitas.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/palomitas.dir/build: libpalomitas.so
.PHONY : CMakeFiles/palomitas.dir/build

CMakeFiles/palomitas.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/palomitas.dir/cmake_clean.cmake
.PHONY : CMakeFiles/palomitas.dir/clean

CMakeFiles/palomitas.dir/depend:
	cd /home/fmrico/ros/ros2/asr24_ws/src/ASR_2024/tutorial_cmake/step_2/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/fmrico/ros/ros2/asr24_ws/src/ASR_2024/tutorial_cmake/step_2 /home/fmrico/ros/ros2/asr24_ws/src/ASR_2024/tutorial_cmake/step_2 /home/fmrico/ros/ros2/asr24_ws/src/ASR_2024/tutorial_cmake/step_2/build /home/fmrico/ros/ros2/asr24_ws/src/ASR_2024/tutorial_cmake/step_2/build /home/fmrico/ros/ros2/asr24_ws/src/ASR_2024/tutorial_cmake/step_2/build/CMakeFiles/palomitas.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/palomitas.dir/depend

