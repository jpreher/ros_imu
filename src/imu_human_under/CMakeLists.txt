cmake_minimum_required(VERSION 2.8.3)
project(imu_human_under)
add_definitions(-std=c++0x)

set(IMU_COMMON_PATH "../imu_common")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
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
#  LIBRARIES imu_node
#  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs std_srvs
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

add_executable(mpu_human_under src/imu_human_under.cpp)
target_link_libraries(mpu_human_under ${catkin_LIBRARIES} imu_common)
add_dependencies(mpu_human_under mpu_gencpp)