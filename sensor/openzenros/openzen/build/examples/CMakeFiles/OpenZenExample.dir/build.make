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
CMAKE_SOURCE_DIR = /home/hayashi/worksp/yaskawa/src/sensor/openzenros/openzen

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/hayashi/worksp/yaskawa/src/sensor/openzenros/openzen/build

# Include any dependencies generated for this target.
include examples/CMakeFiles/OpenZenExample.dir/depend.make

# Include the progress variables for this target.
include examples/CMakeFiles/OpenZenExample.dir/progress.make

# Include the compile flags for this target's objects.
include examples/CMakeFiles/OpenZenExample.dir/flags.make

examples/CMakeFiles/OpenZenExample.dir/main.cpp.o: examples/CMakeFiles/OpenZenExample.dir/flags.make
examples/CMakeFiles/OpenZenExample.dir/main.cpp.o: ../examples/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hayashi/worksp/yaskawa/src/sensor/openzenros/openzen/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object examples/CMakeFiles/OpenZenExample.dir/main.cpp.o"
	cd /home/hayashi/worksp/yaskawa/src/sensor/openzenros/openzen/build/examples && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/OpenZenExample.dir/main.cpp.o -c /home/hayashi/worksp/yaskawa/src/sensor/openzenros/openzen/examples/main.cpp

examples/CMakeFiles/OpenZenExample.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/OpenZenExample.dir/main.cpp.i"
	cd /home/hayashi/worksp/yaskawa/src/sensor/openzenros/openzen/build/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/hayashi/worksp/yaskawa/src/sensor/openzenros/openzen/examples/main.cpp > CMakeFiles/OpenZenExample.dir/main.cpp.i

examples/CMakeFiles/OpenZenExample.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/OpenZenExample.dir/main.cpp.s"
	cd /home/hayashi/worksp/yaskawa/src/sensor/openzenros/openzen/build/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/hayashi/worksp/yaskawa/src/sensor/openzenros/openzen/examples/main.cpp -o CMakeFiles/OpenZenExample.dir/main.cpp.s

examples/CMakeFiles/OpenZenExample.dir/main.cpp.o.requires:

.PHONY : examples/CMakeFiles/OpenZenExample.dir/main.cpp.o.requires

examples/CMakeFiles/OpenZenExample.dir/main.cpp.o.provides: examples/CMakeFiles/OpenZenExample.dir/main.cpp.o.requires
	$(MAKE) -f examples/CMakeFiles/OpenZenExample.dir/build.make examples/CMakeFiles/OpenZenExample.dir/main.cpp.o.provides.build
.PHONY : examples/CMakeFiles/OpenZenExample.dir/main.cpp.o.provides

examples/CMakeFiles/OpenZenExample.dir/main.cpp.o.provides.build: examples/CMakeFiles/OpenZenExample.dir/main.cpp.o


# Object files for target OpenZenExample
OpenZenExample_OBJECTS = \
"CMakeFiles/OpenZenExample.dir/main.cpp.o"

# External object files for target OpenZenExample
OpenZenExample_EXTERNAL_OBJECTS =

examples/OpenZenExample: examples/CMakeFiles/OpenZenExample.dir/main.cpp.o
examples/OpenZenExample: examples/CMakeFiles/OpenZenExample.dir/build.make
examples/OpenZenExample: libOpenZen.so
examples/OpenZenExample: examples/CMakeFiles/OpenZenExample.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/hayashi/worksp/yaskawa/src/sensor/openzenros/openzen/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable OpenZenExample"
	cd /home/hayashi/worksp/yaskawa/src/sensor/openzenros/openzen/build/examples && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/OpenZenExample.dir/link.txt --verbose=$(VERBOSE)
	cd /home/hayashi/worksp/yaskawa/src/sensor/openzenros/openzen/build/examples && /usr/bin/cmake -E copy_if_different /home/hayashi/worksp/yaskawa/src/sensor/openzenros/openzen/build/libOpenZen.so /home/hayashi/worksp/yaskawa/src/sensor/openzenros/openzen/build/examples

# Rule to build all files generated by this target.
examples/CMakeFiles/OpenZenExample.dir/build: examples/OpenZenExample

.PHONY : examples/CMakeFiles/OpenZenExample.dir/build

examples/CMakeFiles/OpenZenExample.dir/requires: examples/CMakeFiles/OpenZenExample.dir/main.cpp.o.requires

.PHONY : examples/CMakeFiles/OpenZenExample.dir/requires

examples/CMakeFiles/OpenZenExample.dir/clean:
	cd /home/hayashi/worksp/yaskawa/src/sensor/openzenros/openzen/build/examples && $(CMAKE_COMMAND) -P CMakeFiles/OpenZenExample.dir/cmake_clean.cmake
.PHONY : examples/CMakeFiles/OpenZenExample.dir/clean

examples/CMakeFiles/OpenZenExample.dir/depend:
	cd /home/hayashi/worksp/yaskawa/src/sensor/openzenros/openzen/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/hayashi/worksp/yaskawa/src/sensor/openzenros/openzen /home/hayashi/worksp/yaskawa/src/sensor/openzenros/openzen/examples /home/hayashi/worksp/yaskawa/src/sensor/openzenros/openzen/build /home/hayashi/worksp/yaskawa/src/sensor/openzenros/openzen/build/examples /home/hayashi/worksp/yaskawa/src/sensor/openzenros/openzen/build/examples/CMakeFiles/OpenZenExample.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : examples/CMakeFiles/OpenZenExample.dir/depend

