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
CMAKE_COMMAND = /home/vasko/clion-2018.1.5/bin/cmake/bin/cmake

# The command to remove a file.
RM = /home/vasko/clion-2018.1.5/bin/cmake/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/vasko/CLionProjects/FCL_STATE_CHECK

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/vasko/CLionProjects/FCL_STATE_CHECK/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/FCL_STATE_CHECK.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/FCL_STATE_CHECK.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/FCL_STATE_CHECK.dir/flags.make

CMakeFiles/FCL_STATE_CHECK.dir/main.cpp.o: CMakeFiles/FCL_STATE_CHECK.dir/flags.make
CMakeFiles/FCL_STATE_CHECK.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/vasko/CLionProjects/FCL_STATE_CHECK/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/FCL_STATE_CHECK.dir/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/FCL_STATE_CHECK.dir/main.cpp.o -c /home/vasko/CLionProjects/FCL_STATE_CHECK/main.cpp

CMakeFiles/FCL_STATE_CHECK.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/FCL_STATE_CHECK.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/vasko/CLionProjects/FCL_STATE_CHECK/main.cpp > CMakeFiles/FCL_STATE_CHECK.dir/main.cpp.i

CMakeFiles/FCL_STATE_CHECK.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/FCL_STATE_CHECK.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/vasko/CLionProjects/FCL_STATE_CHECK/main.cpp -o CMakeFiles/FCL_STATE_CHECK.dir/main.cpp.s

CMakeFiles/FCL_STATE_CHECK.dir/main.cpp.o.requires:

.PHONY : CMakeFiles/FCL_STATE_CHECK.dir/main.cpp.o.requires

CMakeFiles/FCL_STATE_CHECK.dir/main.cpp.o.provides: CMakeFiles/FCL_STATE_CHECK.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/FCL_STATE_CHECK.dir/build.make CMakeFiles/FCL_STATE_CHECK.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/FCL_STATE_CHECK.dir/main.cpp.o.provides

CMakeFiles/FCL_STATE_CHECK.dir/main.cpp.o.provides.build: CMakeFiles/FCL_STATE_CHECK.dir/main.cpp.o


CMakeFiles/FCL_STATE_CHECK.dir/FCL_ifStateValid.cpp.o: CMakeFiles/FCL_STATE_CHECK.dir/flags.make
CMakeFiles/FCL_STATE_CHECK.dir/FCL_ifStateValid.cpp.o: ../FCL_ifStateValid.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/vasko/CLionProjects/FCL_STATE_CHECK/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/FCL_STATE_CHECK.dir/FCL_ifStateValid.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/FCL_STATE_CHECK.dir/FCL_ifStateValid.cpp.o -c /home/vasko/CLionProjects/FCL_STATE_CHECK/FCL_ifStateValid.cpp

CMakeFiles/FCL_STATE_CHECK.dir/FCL_ifStateValid.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/FCL_STATE_CHECK.dir/FCL_ifStateValid.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/vasko/CLionProjects/FCL_STATE_CHECK/FCL_ifStateValid.cpp > CMakeFiles/FCL_STATE_CHECK.dir/FCL_ifStateValid.cpp.i

CMakeFiles/FCL_STATE_CHECK.dir/FCL_ifStateValid.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/FCL_STATE_CHECK.dir/FCL_ifStateValid.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/vasko/CLionProjects/FCL_STATE_CHECK/FCL_ifStateValid.cpp -o CMakeFiles/FCL_STATE_CHECK.dir/FCL_ifStateValid.cpp.s

CMakeFiles/FCL_STATE_CHECK.dir/FCL_ifStateValid.cpp.o.requires:

.PHONY : CMakeFiles/FCL_STATE_CHECK.dir/FCL_ifStateValid.cpp.o.requires

CMakeFiles/FCL_STATE_CHECK.dir/FCL_ifStateValid.cpp.o.provides: CMakeFiles/FCL_STATE_CHECK.dir/FCL_ifStateValid.cpp.o.requires
	$(MAKE) -f CMakeFiles/FCL_STATE_CHECK.dir/build.make CMakeFiles/FCL_STATE_CHECK.dir/FCL_ifStateValid.cpp.o.provides.build
.PHONY : CMakeFiles/FCL_STATE_CHECK.dir/FCL_ifStateValid.cpp.o.provides

CMakeFiles/FCL_STATE_CHECK.dir/FCL_ifStateValid.cpp.o.provides.build: CMakeFiles/FCL_STATE_CHECK.dir/FCL_ifStateValid.cpp.o


CMakeFiles/FCL_STATE_CHECK.dir/Get_OpenMesh_data.cpp.o: CMakeFiles/FCL_STATE_CHECK.dir/flags.make
CMakeFiles/FCL_STATE_CHECK.dir/Get_OpenMesh_data.cpp.o: ../Get_OpenMesh_data.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/vasko/CLionProjects/FCL_STATE_CHECK/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/FCL_STATE_CHECK.dir/Get_OpenMesh_data.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/FCL_STATE_CHECK.dir/Get_OpenMesh_data.cpp.o -c /home/vasko/CLionProjects/FCL_STATE_CHECK/Get_OpenMesh_data.cpp

CMakeFiles/FCL_STATE_CHECK.dir/Get_OpenMesh_data.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/FCL_STATE_CHECK.dir/Get_OpenMesh_data.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/vasko/CLionProjects/FCL_STATE_CHECK/Get_OpenMesh_data.cpp > CMakeFiles/FCL_STATE_CHECK.dir/Get_OpenMesh_data.cpp.i

CMakeFiles/FCL_STATE_CHECK.dir/Get_OpenMesh_data.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/FCL_STATE_CHECK.dir/Get_OpenMesh_data.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/vasko/CLionProjects/FCL_STATE_CHECK/Get_OpenMesh_data.cpp -o CMakeFiles/FCL_STATE_CHECK.dir/Get_OpenMesh_data.cpp.s

CMakeFiles/FCL_STATE_CHECK.dir/Get_OpenMesh_data.cpp.o.requires:

.PHONY : CMakeFiles/FCL_STATE_CHECK.dir/Get_OpenMesh_data.cpp.o.requires

CMakeFiles/FCL_STATE_CHECK.dir/Get_OpenMesh_data.cpp.o.provides: CMakeFiles/FCL_STATE_CHECK.dir/Get_OpenMesh_data.cpp.o.requires
	$(MAKE) -f CMakeFiles/FCL_STATE_CHECK.dir/build.make CMakeFiles/FCL_STATE_CHECK.dir/Get_OpenMesh_data.cpp.o.provides.build
.PHONY : CMakeFiles/FCL_STATE_CHECK.dir/Get_OpenMesh_data.cpp.o.provides

CMakeFiles/FCL_STATE_CHECK.dir/Get_OpenMesh_data.cpp.o.provides.build: CMakeFiles/FCL_STATE_CHECK.dir/Get_OpenMesh_data.cpp.o


# Object files for target FCL_STATE_CHECK
FCL_STATE_CHECK_OBJECTS = \
"CMakeFiles/FCL_STATE_CHECK.dir/main.cpp.o" \
"CMakeFiles/FCL_STATE_CHECK.dir/FCL_ifStateValid.cpp.o" \
"CMakeFiles/FCL_STATE_CHECK.dir/Get_OpenMesh_data.cpp.o"

# External object files for target FCL_STATE_CHECK
FCL_STATE_CHECK_EXTERNAL_OBJECTS =

FCL_STATE_CHECK: CMakeFiles/FCL_STATE_CHECK.dir/main.cpp.o
FCL_STATE_CHECK: CMakeFiles/FCL_STATE_CHECK.dir/FCL_ifStateValid.cpp.o
FCL_STATE_CHECK: CMakeFiles/FCL_STATE_CHECK.dir/Get_OpenMesh_data.cpp.o
FCL_STATE_CHECK: CMakeFiles/FCL_STATE_CHECK.dir/build.make
FCL_STATE_CHECK: /home/vasko/omplapp/lib/libompl.so
FCL_STATE_CHECK: /usr/local/lib/libfcl.so.0.5
FCL_STATE_CHECK: CMakeFiles/FCL_STATE_CHECK.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/vasko/CLionProjects/FCL_STATE_CHECK/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable FCL_STATE_CHECK"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/FCL_STATE_CHECK.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/FCL_STATE_CHECK.dir/build: FCL_STATE_CHECK

.PHONY : CMakeFiles/FCL_STATE_CHECK.dir/build

CMakeFiles/FCL_STATE_CHECK.dir/requires: CMakeFiles/FCL_STATE_CHECK.dir/main.cpp.o.requires
CMakeFiles/FCL_STATE_CHECK.dir/requires: CMakeFiles/FCL_STATE_CHECK.dir/FCL_ifStateValid.cpp.o.requires
CMakeFiles/FCL_STATE_CHECK.dir/requires: CMakeFiles/FCL_STATE_CHECK.dir/Get_OpenMesh_data.cpp.o.requires

.PHONY : CMakeFiles/FCL_STATE_CHECK.dir/requires

CMakeFiles/FCL_STATE_CHECK.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/FCL_STATE_CHECK.dir/cmake_clean.cmake
.PHONY : CMakeFiles/FCL_STATE_CHECK.dir/clean

CMakeFiles/FCL_STATE_CHECK.dir/depend:
	cd /home/vasko/CLionProjects/FCL_STATE_CHECK/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/vasko/CLionProjects/FCL_STATE_CHECK /home/vasko/CLionProjects/FCL_STATE_CHECK /home/vasko/CLionProjects/FCL_STATE_CHECK/cmake-build-debug /home/vasko/CLionProjects/FCL_STATE_CHECK/cmake-build-debug /home/vasko/CLionProjects/FCL_STATE_CHECK/cmake-build-debug/CMakeFiles/FCL_STATE_CHECK.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/FCL_STATE_CHECK.dir/depend

