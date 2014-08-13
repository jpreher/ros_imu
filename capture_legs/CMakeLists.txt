cmake_minimum_required(VERSION 2.8.3)
project(capture_legs)
add_definitions(-std=c++0x)

set(IMU_COMMON_PATH "../imu_common")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  std_srvs
  imu_common
)

include_directories(
	${IMU_COMMON_PATH}
	${IMU_COMMON_PATH}/include
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
#  INCLUDE_DIRS include
  LIBRARIES capture_legs
  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs imu_common
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(capture_legs src/capture_legs.cpp)
target_link_libraries(capture_legs ${catkin_LIBRARIES} imu_common)
add_dependencies(capture_legs cap_gencpp)

add_executable(proxi_tau src/proxi_tau.cpp)
target_link_libraries(proxi_tau ${catkin_LIBRARIES} imu_common)