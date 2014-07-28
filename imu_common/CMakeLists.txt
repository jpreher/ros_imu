cmake_minimum_required(VERSION 2.8.3)
project(imu_common)
add_definitions(-std=c++0x)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
  std_msgs
  sensor_msgs
)

## Generate messages in the 'msg' folder
 add_message_files(
   DIRECTORY msg
   FILES
   imu.msg
 )

 ## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
 )


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES imu_common
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(imu_common src/MPU9150.cpp include/MPU9150.h 
	src/BBB_I2C.cpp include/BBB_I2C.h 
	src/quaternion_util.cpp include/quaternion_util.h
	include/butterworth_util.h include/MPU_RegMap.h
	include/limits.hpp include/common_assert.hpp
  include/pca9547.h src/pca9547.cpp
  include/ambcap.h src/ambcap.cpp)

add_dependencies(imu_common imu_common_gencpp)





