# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/amontano/quadWS/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/amontano/quadWS/build

# Include any dependencies generated for this target.
include quad/CMakeFiles/botLib.dir/depend.make

# Include the progress variables for this target.
include quad/CMakeFiles/botLib.dir/progress.make

# Include the compile flags for this target's objects.
include quad/CMakeFiles/botLib.dir/flags.make

quad/CMakeFiles/botLib.dir/botLib.cpp.o: quad/CMakeFiles/botLib.dir/flags.make
quad/CMakeFiles/botLib.dir/botLib.cpp.o: /home/amontano/quadWS/src/quad/botLib.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/amontano/quadWS/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object quad/CMakeFiles/botLib.dir/botLib.cpp.o"
	cd /home/amontano/quadWS/build/quad && /usr/lib/ccache/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/botLib.dir/botLib.cpp.o -c /home/amontano/quadWS/src/quad/botLib.cpp

quad/CMakeFiles/botLib.dir/botLib.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/botLib.dir/botLib.cpp.i"
	cd /home/amontano/quadWS/build/quad && /usr/lib/ccache/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/amontano/quadWS/src/quad/botLib.cpp > CMakeFiles/botLib.dir/botLib.cpp.i

quad/CMakeFiles/botLib.dir/botLib.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/botLib.dir/botLib.cpp.s"
	cd /home/amontano/quadWS/build/quad && /usr/lib/ccache/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/amontano/quadWS/src/quad/botLib.cpp -o CMakeFiles/botLib.dir/botLib.cpp.s

quad/CMakeFiles/botLib.dir/botLib.cpp.o.requires:

.PHONY : quad/CMakeFiles/botLib.dir/botLib.cpp.o.requires

quad/CMakeFiles/botLib.dir/botLib.cpp.o.provides: quad/CMakeFiles/botLib.dir/botLib.cpp.o.requires
	$(MAKE) -f quad/CMakeFiles/botLib.dir/build.make quad/CMakeFiles/botLib.dir/botLib.cpp.o.provides.build
.PHONY : quad/CMakeFiles/botLib.dir/botLib.cpp.o.provides

quad/CMakeFiles/botLib.dir/botLib.cpp.o.provides.build: quad/CMakeFiles/botLib.dir/botLib.cpp.o


# Object files for target botLib
botLib_OBJECTS = \
"CMakeFiles/botLib.dir/botLib.cpp.o"

# External object files for target botLib
botLib_EXTERNAL_OBJECTS =

/home/amontano/quadWS/devel/lib/libbotLib.so: quad/CMakeFiles/botLib.dir/botLib.cpp.o
/home/amontano/quadWS/devel/lib/libbotLib.so: quad/CMakeFiles/botLib.dir/build.make
/home/amontano/quadWS/devel/lib/libbotLib.so: quad/CMakeFiles/botLib.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/amontano/quadWS/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library /home/amontano/quadWS/devel/lib/libbotLib.so"
	cd /home/amontano/quadWS/build/quad && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/botLib.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
quad/CMakeFiles/botLib.dir/build: /home/amontano/quadWS/devel/lib/libbotLib.so

.PHONY : quad/CMakeFiles/botLib.dir/build

quad/CMakeFiles/botLib.dir/requires: quad/CMakeFiles/botLib.dir/botLib.cpp.o.requires

.PHONY : quad/CMakeFiles/botLib.dir/requires

quad/CMakeFiles/botLib.dir/clean:
	cd /home/amontano/quadWS/build/quad && $(CMAKE_COMMAND) -P CMakeFiles/botLib.dir/cmake_clean.cmake
.PHONY : quad/CMakeFiles/botLib.dir/clean

quad/CMakeFiles/botLib.dir/depend:
	cd /home/amontano/quadWS/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/amontano/quadWS/src /home/amontano/quadWS/src/quad /home/amontano/quadWS/build /home/amontano/quadWS/build/quad /home/amontano/quadWS/build/quad/CMakeFiles/botLib.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : quad/CMakeFiles/botLib.dir/depend
