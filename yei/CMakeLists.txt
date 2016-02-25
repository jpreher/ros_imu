cmake_minimum_required(VERSION 2.8.3)
project(yei)

SET(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++11 -pthread")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  message_generation
  std_msgs
  sensor_msgs
)

add_message_files(
   FILES yei_msg.msg
)

 ## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
 )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES yei_lib
  CATKIN_DEPENDS message_runtime
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(yei_lib
  src/yei_threespace.cpp  include/yei_threespace.hpp
  include/yei_commands.hpp
  src/test.cpp
  ${catkin_Libraries}
)
add_dependencies(yei_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(sensor_test src/test.cpp)
add_dependencies(sensor_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(sensor_test yei_lib ${catkin_LIBRARIES})
