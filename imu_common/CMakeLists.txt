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
  eigen_utilities
  roslib_utilities
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
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    yei
    yaml_eigen_utilities
    roslib_utilities
  DEPENDS
    eigen_utilities
)

# Find Eigen3
find_package(PkgConfig)
pkg_check_modules(EIGEN3 eigen3)
add_definitions(${EIGEN3_CFLAGS_OTHER})
include_directories(${EIGEN3_INCLUDE_DIRS})

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  #include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
  gen
  gen/include
)

include(gen/sources.cmake)

## Declare a cpp library
add_library(${PROJECT_NAME}
  src/EKF.cpp
  src/estimator.cpp
  src/test.cpp
  ${SRCS}
)

add_dependencies(imu_common ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(ekf_chain_test src/test.cpp)
add_dependencies(ekf_chain_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ekf_chain_test yei ${PROJECT_NAME} ${catkin_LIBRARIES})

#add_executable(sensor_test src/test.cpp)
#add_dependencies(sensor_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(sensor_test yei_lib ${catkin_LIBRARIES})

