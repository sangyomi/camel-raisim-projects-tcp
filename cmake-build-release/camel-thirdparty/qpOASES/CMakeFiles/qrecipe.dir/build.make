# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.25

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
CMAKE_COMMAND = /snap/clion/235/bin/cmake/linux/x64/bin/cmake

# The command to remove a file.
RM = /snap/clion/235/bin/cmake/linux/x64/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/sangjun/camel-raisim-projects

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/sangjun/camel-raisim-projects/cmake-build-release

# Include any dependencies generated for this target.
include camel-thirdparty/qpOASES/CMakeFiles/qrecipe.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include camel-thirdparty/qpOASES/CMakeFiles/qrecipe.dir/compiler_depend.make

# Include the progress variables for this target.
include camel-thirdparty/qpOASES/CMakeFiles/qrecipe.dir/progress.make

# Include the compile flags for this target's objects.
include camel-thirdparty/qpOASES/CMakeFiles/qrecipe.dir/flags.make

camel-thirdparty/qpOASES/CMakeFiles/qrecipe.dir/examples/qrecipe.cpp.o: camel-thirdparty/qpOASES/CMakeFiles/qrecipe.dir/flags.make
camel-thirdparty/qpOASES/CMakeFiles/qrecipe.dir/examples/qrecipe.cpp.o: /home/sangjun/camel-raisim-projects/camel-thirdparty/qpOASES/examples/qrecipe.cpp
camel-thirdparty/qpOASES/CMakeFiles/qrecipe.dir/examples/qrecipe.cpp.o: camel-thirdparty/qpOASES/CMakeFiles/qrecipe.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/sangjun/camel-raisim-projects/cmake-build-release/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object camel-thirdparty/qpOASES/CMakeFiles/qrecipe.dir/examples/qrecipe.cpp.o"
	cd /home/sangjun/camel-raisim-projects/cmake-build-release/camel-thirdparty/qpOASES && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT camel-thirdparty/qpOASES/CMakeFiles/qrecipe.dir/examples/qrecipe.cpp.o -MF CMakeFiles/qrecipe.dir/examples/qrecipe.cpp.o.d -o CMakeFiles/qrecipe.dir/examples/qrecipe.cpp.o -c /home/sangjun/camel-raisim-projects/camel-thirdparty/qpOASES/examples/qrecipe.cpp

camel-thirdparty/qpOASES/CMakeFiles/qrecipe.dir/examples/qrecipe.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/qrecipe.dir/examples/qrecipe.cpp.i"
	cd /home/sangjun/camel-raisim-projects/cmake-build-release/camel-thirdparty/qpOASES && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/sangjun/camel-raisim-projects/camel-thirdparty/qpOASES/examples/qrecipe.cpp > CMakeFiles/qrecipe.dir/examples/qrecipe.cpp.i

camel-thirdparty/qpOASES/CMakeFiles/qrecipe.dir/examples/qrecipe.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/qrecipe.dir/examples/qrecipe.cpp.s"
	cd /home/sangjun/camel-raisim-projects/cmake-build-release/camel-thirdparty/qpOASES && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/sangjun/camel-raisim-projects/camel-thirdparty/qpOASES/examples/qrecipe.cpp -o CMakeFiles/qrecipe.dir/examples/qrecipe.cpp.s

# Object files for target qrecipe
qrecipe_OBJECTS = \
"CMakeFiles/qrecipe.dir/examples/qrecipe.cpp.o"

# External object files for target qrecipe
qrecipe_EXTERNAL_OBJECTS =

camel-thirdparty/qpOASES/bin/qrecipe: camel-thirdparty/qpOASES/CMakeFiles/qrecipe.dir/examples/qrecipe.cpp.o
camel-thirdparty/qpOASES/bin/qrecipe: camel-thirdparty/qpOASES/CMakeFiles/qrecipe.dir/build.make
camel-thirdparty/qpOASES/bin/qrecipe: camel-thirdparty/qpOASES/libs/libqpOASES.a
camel-thirdparty/qpOASES/bin/qrecipe: camel-thirdparty/qpOASES/CMakeFiles/qrecipe.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/sangjun/camel-raisim-projects/cmake-build-release/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable bin/qrecipe"
	cd /home/sangjun/camel-raisim-projects/cmake-build-release/camel-thirdparty/qpOASES && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/qrecipe.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
camel-thirdparty/qpOASES/CMakeFiles/qrecipe.dir/build: camel-thirdparty/qpOASES/bin/qrecipe
.PHONY : camel-thirdparty/qpOASES/CMakeFiles/qrecipe.dir/build

camel-thirdparty/qpOASES/CMakeFiles/qrecipe.dir/clean:
	cd /home/sangjun/camel-raisim-projects/cmake-build-release/camel-thirdparty/qpOASES && $(CMAKE_COMMAND) -P CMakeFiles/qrecipe.dir/cmake_clean.cmake
.PHONY : camel-thirdparty/qpOASES/CMakeFiles/qrecipe.dir/clean

camel-thirdparty/qpOASES/CMakeFiles/qrecipe.dir/depend:
	cd /home/sangjun/camel-raisim-projects/cmake-build-release && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/sangjun/camel-raisim-projects /home/sangjun/camel-raisim-projects/camel-thirdparty/qpOASES /home/sangjun/camel-raisim-projects/cmake-build-release /home/sangjun/camel-raisim-projects/cmake-build-release/camel-thirdparty/qpOASES /home/sangjun/camel-raisim-projects/cmake-build-release/camel-thirdparty/qpOASES/CMakeFiles/qrecipe.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : camel-thirdparty/qpOASES/CMakeFiles/qrecipe.dir/depend

