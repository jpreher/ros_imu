# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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
CMAKE_SOURCE_DIR = /home/jake/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jake/catkin_ws/build

# Include any dependencies generated for this target.
include imu_node/CMakeFiles/mpu_driver_lib.dir/depend.make

# Include the progress variables for this target.
include imu_node/CMakeFiles/mpu_driver_lib.dir/progress.make

# Include the compile flags for this target's objects.
include imu_node/CMakeFiles/mpu_driver_lib.dir/flags.make

imu_node/CMakeFiles/mpu_driver_lib.dir/src/MPU9150.cpp.o: imu_node/CMakeFiles/mpu_driver_lib.dir/flags.make
imu_node/CMakeFiles/mpu_driver_lib.dir/src/MPU9150.cpp.o: /home/jake/catkin_ws/src/imu_node/src/MPU9150.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/jake/catkin_ws/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object imu_node/CMakeFiles/mpu_driver_lib.dir/src/MPU9150.cpp.o"
	cd /home/jake/catkin_ws/build/imu_node && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/mpu_driver_lib.dir/src/MPU9150.cpp.o -c /home/jake/catkin_ws/src/imu_node/src/MPU9150.cpp

imu_node/CMakeFiles/mpu_driver_lib.dir/src/MPU9150.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mpu_driver_lib.dir/src/MPU9150.cpp.i"
	cd /home/jake/catkin_ws/build/imu_node && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/jake/catkin_ws/src/imu_node/src/MPU9150.cpp > CMakeFiles/mpu_driver_lib.dir/src/MPU9150.cpp.i

imu_node/CMakeFiles/mpu_driver_lib.dir/src/MPU9150.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mpu_driver_lib.dir/src/MPU9150.cpp.s"
	cd /home/jake/catkin_ws/build/imu_node && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/jake/catkin_ws/src/imu_node/src/MPU9150.cpp -o CMakeFiles/mpu_driver_lib.dir/src/MPU9150.cpp.s

imu_node/CMakeFiles/mpu_driver_lib.dir/src/MPU9150.cpp.o.requires:
.PHONY : imu_node/CMakeFiles/mpu_driver_lib.dir/src/MPU9150.cpp.o.requires

imu_node/CMakeFiles/mpu_driver_lib.dir/src/MPU9150.cpp.o.provides: imu_node/CMakeFiles/mpu_driver_lib.dir/src/MPU9150.cpp.o.requires
	$(MAKE) -f imu_node/CMakeFiles/mpu_driver_lib.dir/build.make imu_node/CMakeFiles/mpu_driver_lib.dir/src/MPU9150.cpp.o.provides.build
.PHONY : imu_node/CMakeFiles/mpu_driver_lib.dir/src/MPU9150.cpp.o.provides

imu_node/CMakeFiles/mpu_driver_lib.dir/src/MPU9150.cpp.o.provides.build: imu_node/CMakeFiles/mpu_driver_lib.dir/src/MPU9150.cpp.o

imu_node/CMakeFiles/mpu_driver_lib.dir/src/BBB_I2C.cpp.o: imu_node/CMakeFiles/mpu_driver_lib.dir/flags.make
imu_node/CMakeFiles/mpu_driver_lib.dir/src/BBB_I2C.cpp.o: /home/jake/catkin_ws/src/imu_node/src/BBB_I2C.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/jake/catkin_ws/build/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object imu_node/CMakeFiles/mpu_driver_lib.dir/src/BBB_I2C.cpp.o"
	cd /home/jake/catkin_ws/build/imu_node && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/mpu_driver_lib.dir/src/BBB_I2C.cpp.o -c /home/jake/catkin_ws/src/imu_node/src/BBB_I2C.cpp

imu_node/CMakeFiles/mpu_driver_lib.dir/src/BBB_I2C.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mpu_driver_lib.dir/src/BBB_I2C.cpp.i"
	cd /home/jake/catkin_ws/build/imu_node && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/jake/catkin_ws/src/imu_node/src/BBB_I2C.cpp > CMakeFiles/mpu_driver_lib.dir/src/BBB_I2C.cpp.i

imu_node/CMakeFiles/mpu_driver_lib.dir/src/BBB_I2C.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mpu_driver_lib.dir/src/BBB_I2C.cpp.s"
	cd /home/jake/catkin_ws/build/imu_node && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/jake/catkin_ws/src/imu_node/src/BBB_I2C.cpp -o CMakeFiles/mpu_driver_lib.dir/src/BBB_I2C.cpp.s

imu_node/CMakeFiles/mpu_driver_lib.dir/src/BBB_I2C.cpp.o.requires:
.PHONY : imu_node/CMakeFiles/mpu_driver_lib.dir/src/BBB_I2C.cpp.o.requires

imu_node/CMakeFiles/mpu_driver_lib.dir/src/BBB_I2C.cpp.o.provides: imu_node/CMakeFiles/mpu_driver_lib.dir/src/BBB_I2C.cpp.o.requires
	$(MAKE) -f imu_node/CMakeFiles/mpu_driver_lib.dir/build.make imu_node/CMakeFiles/mpu_driver_lib.dir/src/BBB_I2C.cpp.o.provides.build
.PHONY : imu_node/CMakeFiles/mpu_driver_lib.dir/src/BBB_I2C.cpp.o.provides

imu_node/CMakeFiles/mpu_driver_lib.dir/src/BBB_I2C.cpp.o.provides.build: imu_node/CMakeFiles/mpu_driver_lib.dir/src/BBB_I2C.cpp.o

imu_node/CMakeFiles/mpu_driver_lib.dir/src/quaternion_util.cpp.o: imu_node/CMakeFiles/mpu_driver_lib.dir/flags.make
imu_node/CMakeFiles/mpu_driver_lib.dir/src/quaternion_util.cpp.o: /home/jake/catkin_ws/src/imu_node/src/quaternion_util.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/jake/catkin_ws/build/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object imu_node/CMakeFiles/mpu_driver_lib.dir/src/quaternion_util.cpp.o"
	cd /home/jake/catkin_ws/build/imu_node && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/mpu_driver_lib.dir/src/quaternion_util.cpp.o -c /home/jake/catkin_ws/src/imu_node/src/quaternion_util.cpp

imu_node/CMakeFiles/mpu_driver_lib.dir/src/quaternion_util.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mpu_driver_lib.dir/src/quaternion_util.cpp.i"
	cd /home/jake/catkin_ws/build/imu_node && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/jake/catkin_ws/src/imu_node/src/quaternion_util.cpp > CMakeFiles/mpu_driver_lib.dir/src/quaternion_util.cpp.i

imu_node/CMakeFiles/mpu_driver_lib.dir/src/quaternion_util.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mpu_driver_lib.dir/src/quaternion_util.cpp.s"
	cd /home/jake/catkin_ws/build/imu_node && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/jake/catkin_ws/src/imu_node/src/quaternion_util.cpp -o CMakeFiles/mpu_driver_lib.dir/src/quaternion_util.cpp.s

imu_node/CMakeFiles/mpu_driver_lib.dir/src/quaternion_util.cpp.o.requires:
.PHONY : imu_node/CMakeFiles/mpu_driver_lib.dir/src/quaternion_util.cpp.o.requires

imu_node/CMakeFiles/mpu_driver_lib.dir/src/quaternion_util.cpp.o.provides: imu_node/CMakeFiles/mpu_driver_lib.dir/src/quaternion_util.cpp.o.requires
	$(MAKE) -f imu_node/CMakeFiles/mpu_driver_lib.dir/build.make imu_node/CMakeFiles/mpu_driver_lib.dir/src/quaternion_util.cpp.o.provides.build
.PHONY : imu_node/CMakeFiles/mpu_driver_lib.dir/src/quaternion_util.cpp.o.provides

imu_node/CMakeFiles/mpu_driver_lib.dir/src/quaternion_util.cpp.o.provides.build: imu_node/CMakeFiles/mpu_driver_lib.dir/src/quaternion_util.cpp.o

# Object files for target mpu_driver_lib
mpu_driver_lib_OBJECTS = \
"CMakeFiles/mpu_driver_lib.dir/src/MPU9150.cpp.o" \
"CMakeFiles/mpu_driver_lib.dir/src/BBB_I2C.cpp.o" \
"CMakeFiles/mpu_driver_lib.dir/src/quaternion_util.cpp.o"

# External object files for target mpu_driver_lib
mpu_driver_lib_EXTERNAL_OBJECTS =

/home/jake/catkin_ws/devel/lib/libmpu_driver_lib.so: imu_node/CMakeFiles/mpu_driver_lib.dir/src/MPU9150.cpp.o
/home/jake/catkin_ws/devel/lib/libmpu_driver_lib.so: imu_node/CMakeFiles/mpu_driver_lib.dir/src/BBB_I2C.cpp.o
/home/jake/catkin_ws/devel/lib/libmpu_driver_lib.so: imu_node/CMakeFiles/mpu_driver_lib.dir/src/quaternion_util.cpp.o
/home/jake/catkin_ws/devel/lib/libmpu_driver_lib.so: imu_node/CMakeFiles/mpu_driver_lib.dir/build.make
/home/jake/catkin_ws/devel/lib/libmpu_driver_lib.so: imu_node/CMakeFiles/mpu_driver_lib.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX shared library /home/jake/catkin_ws/devel/lib/libmpu_driver_lib.so"
	cd /home/jake/catkin_ws/build/imu_node && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/mpu_driver_lib.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
imu_node/CMakeFiles/mpu_driver_lib.dir/build: /home/jake/catkin_ws/devel/lib/libmpu_driver_lib.so
.PHONY : imu_node/CMakeFiles/mpu_driver_lib.dir/build

imu_node/CMakeFiles/mpu_driver_lib.dir/requires: imu_node/CMakeFiles/mpu_driver_lib.dir/src/MPU9150.cpp.o.requires
imu_node/CMakeFiles/mpu_driver_lib.dir/requires: imu_node/CMakeFiles/mpu_driver_lib.dir/src/BBB_I2C.cpp.o.requires
imu_node/CMakeFiles/mpu_driver_lib.dir/requires: imu_node/CMakeFiles/mpu_driver_lib.dir/src/quaternion_util.cpp.o.requires
.PHONY : imu_node/CMakeFiles/mpu_driver_lib.dir/requires

imu_node/CMakeFiles/mpu_driver_lib.dir/clean:
	cd /home/jake/catkin_ws/build/imu_node && $(CMAKE_COMMAND) -P CMakeFiles/mpu_driver_lib.dir/cmake_clean.cmake
.PHONY : imu_node/CMakeFiles/mpu_driver_lib.dir/clean

imu_node/CMakeFiles/mpu_driver_lib.dir/depend:
	cd /home/jake/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jake/catkin_ws/src /home/jake/catkin_ws/src/imu_node /home/jake/catkin_ws/build /home/jake/catkin_ws/build/imu_node /home/jake/catkin_ws/build/imu_node/CMakeFiles/mpu_driver_lib.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : imu_node/CMakeFiles/mpu_driver_lib.dir/depend

