cmake_minimum_required(VERSION 2.8.3)
project(imu_common)

add_definitions(-std=c++0x)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  message_generation
  std_msgs
  sensor_msgs
  yei
  yaml_eigen_utilities
)

#add_message_files(
#   FILES yei_msg.msg
#)

 ## Generate added messages and services with any dependencies listed here
 #generate_messages(
 #  DEPENDENCIES
 #  std_msgs
 #  geometry_msgs
 #)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES imu_common
  CATKIN_DEPENDS
    yei
    yaml_eigen_utilities
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(imu_common
  src/EKF.cpp  include/EKF.hpp
  src/estimator.cpp include/estimator.hpp
  ${catkin_Libraries}
)
add_dependencies(imu_common ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#add_executable(sensor_test src/test.cpp)
#add_dependencies(sensor_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(sensor_test yei_lib ${catkin_LIBRARIES})
