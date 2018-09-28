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


# Produce verbose output by default.
VERBOSE = 1

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
CMAKE_COMMAND = /home/vasko/clion-2018.1.5/bin/cmake/bin/cmake

# The command to remove a file.
RM = /home/vasko/clion-2018.1.5/bin/cmake/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/vasko/CLionProjects/Base_OMPL

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/vasko/CLionProjects/Base_OMPL/cmake-build-debug

# Include any dependencies generated for this target.
include Parse_OpenMesh>FCL/CMakeFiles/parse_om_fcl.dir/depend.make

# Include the progress variables for this target.
include Parse_OpenMesh>FCL/CMakeFiles/parse_om_fcl.dir/progress.make

# Include the compile flags for this target's objects.
include Parse_OpenMesh>FCL/CMakeFiles/parse_om_fcl.dir/flags.make

Parse_OpenMesh>FCL/CMakeFiles/parse_om_fcl.dir/get_OpenMesh_data.cpp.o: Parse_OpenMesh>FCL/CMakeFiles/parse_om_fcl.dir/flags.make
Parse_OpenMesh>FCL/CMakeFiles/parse_om_fcl.dir/get_OpenMesh_data.cpp.o: ../Parse_OpenMesh>FCL/get_OpenMesh_data.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/vasko/CLionProjects/Base_OMPL/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object Parse_OpenMesh>FCL/CMakeFiles/parse_om_fcl.dir/get_OpenMesh_data.cpp.o"
	cd "/home/vasko/CLionProjects/Base_OMPL/cmake-build-debug/Parse_OpenMesh>FCL" && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/parse_om_fcl.dir/get_OpenMesh_data.cpp.o -c "/home/vasko/CLionProjects/Base_OMPL/Parse_OpenMesh>FCL/get_OpenMesh_data.cpp"

Parse_OpenMesh>FCL/CMakeFiles/parse_om_fcl.dir/get_OpenMesh_data.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/parse_om_fcl.dir/get_OpenMesh_data.cpp.i"
	cd "/home/vasko/CLionProjects/Base_OMPL/cmake-build-debug/Parse_OpenMesh>FCL" && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/vasko/CLionProjects/Base_OMPL/Parse_OpenMesh>FCL/get_OpenMesh_data.cpp" > CMakeFiles/parse_om_fcl.dir/get_OpenMesh_data.cpp.i

Parse_OpenMesh>FCL/CMakeFiles/parse_om_fcl.dir/get_OpenMesh_data.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/parse_om_fcl.dir/get_OpenMesh_data.cpp.s"
	cd "/home/vasko/CLionProjects/Base_OMPL/cmake-build-debug/Parse_OpenMesh>FCL" && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/vasko/CLionProjects/Base_OMPL/Parse_OpenMesh>FCL/get_OpenMesh_data.cpp" -o CMakeFiles/parse_om_fcl.dir/get_OpenMesh_data.cpp.s

Parse_OpenMesh>FCL/CMakeFiles/parse_om_fcl.dir/get_OpenMesh_data.cpp.o.requires:

.PHONY : Parse_OpenMesh>FCL/CMakeFiles/parse_om_fcl.dir/get_OpenMesh_data.cpp.o.requires

Parse_OpenMesh>FCL/CMakeFiles/parse_om_fcl.dir/get_OpenMesh_data.cpp.o.provides: Parse_OpenMesh>FCL/CMakeFiles/parse_om_fcl.dir/get_OpenMesh_data.cpp.o.requires
	$(MAKE) -f "Parse_OpenMesh>FCL/CMakeFiles/parse_om_fcl.dir/build.make" "Parse_OpenMesh>FCL/CMakeFiles/parse_om_fcl.dir/get_OpenMesh_data.cpp.o.provides.build"
.PHONY : Parse_OpenMesh>FCL/CMakeFiles/parse_om_fcl.dir/get_OpenMesh_data.cpp.o.provides

Parse_OpenMesh>FCL/CMakeFiles/parse_om_fcl.dir/get_OpenMesh_data.cpp.o.provides.build: Parse_OpenMesh>FCL/CMakeFiles/parse_om_fcl.dir/get_OpenMesh_data.cpp.o


# Object files for target parse_om_fcl
parse_om_fcl_OBJECTS = \
"CMakeFiles/parse_om_fcl.dir/get_OpenMesh_data.cpp.o"

# External object files for target parse_om_fcl
parse_om_fcl_EXTERNAL_OBJECTS =

Parse_OpenMesh>FCL/libparse_om_fcl.so: Parse_OpenMesh>FCL/CMakeFiles/parse_om_fcl.dir/get_OpenMesh_data.cpp.o
Parse_OpenMesh>FCL/libparse_om_fcl.so: Parse_OpenMesh>FCL/CMakeFiles/parse_om_fcl.dir/build.make
Parse_OpenMesh>FCL/libparse_om_fcl.so: /usr/local/lib/libfcl.so.0.5
Parse_OpenMesh>FCL/libparse_om_fcl.so: /usr/local/lib/libOpenMeshCore.so
Parse_OpenMesh>FCL/libparse_om_fcl.so: Parse_OpenMesh>FCL/CMakeFiles/parse_om_fcl.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/vasko/CLionProjects/Base_OMPL/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library libparse_om_fcl.so"
	cd "/home/vasko/CLionProjects/Base_OMPL/cmake-build-debug/Parse_OpenMesh>FCL" && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/parse_om_fcl.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
Parse_OpenMesh>FCL/CMakeFiles/parse_om_fcl.dir/build: Parse_OpenMesh>FCL/libparse_om_fcl.so

.PHONY : Parse_OpenMesh>FCL/CMakeFiles/parse_om_fcl.dir/build

Parse_OpenMesh>FCL/CMakeFiles/parse_om_fcl.dir/requires: Parse_OpenMesh>FCL/CMakeFiles/parse_om_fcl.dir/get_OpenMesh_data.cpp.o.requires

.PHONY : Parse_OpenMesh>FCL/CMakeFiles/parse_om_fcl.dir/requires

Parse_OpenMesh>FCL/CMakeFiles/parse_om_fcl.dir/clean:
	cd "/home/vasko/CLionProjects/Base_OMPL/cmake-build-debug/Parse_OpenMesh>FCL" && $(CMAKE_COMMAND) -P CMakeFiles/parse_om_fcl.dir/cmake_clean.cmake
.PHONY : Parse_OpenMesh>FCL/CMakeFiles/parse_om_fcl.dir/clean

Parse_OpenMesh>FCL/CMakeFiles/parse_om_fcl.dir/depend:
	cd /home/vasko/CLionProjects/Base_OMPL/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/vasko/CLionProjects/Base_OMPL "/home/vasko/CLionProjects/Base_OMPL/Parse_OpenMesh>FCL" /home/vasko/CLionProjects/Base_OMPL/cmake-build-debug "/home/vasko/CLionProjects/Base_OMPL/cmake-build-debug/Parse_OpenMesh>FCL" "/home/vasko/CLionProjects/Base_OMPL/cmake-build-debug/Parse_OpenMesh>FCL/CMakeFiles/parse_om_fcl.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : Parse_OpenMesh>FCL/CMakeFiles/parse_om_fcl.dir/depend
