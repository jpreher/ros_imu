cmake_minimum_required(VERSION 2.8.3)
project(yei_sensor)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  message_generation
  sensor_msgs
)

find_package(PythonInterp)
find_package(PythonLibs REQUIRED)

## Add Python Scripts Folder to editor
FILE(GLOB yei_scripts "src/*" "bin/*")
add_custom_target(yei_scripts_location SOURCES ${yei_scripts})

catkin_python_setup()

add_message_files(
   DIRECTORY msg
   FILES
   yei_msg.msg
)

 ## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include ${PYTHON_INCLUDE_DIRS}
  LIBRARIES yei_sensor ${PYTHON_LIBRARIES}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  include
)

## Declare a cpp library
add_library(yei_sensor
  src/yei_wrapper.cpp  include/yei_wrapper.h
  src/yei_driver.cpp   include/yei_driver.h
)
add_dependencies(yei_sensor yei_sensor_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(yei_sensor
  ${catkin_LIBRARIES}
  ${PYTHON_LIBRARIES}
)

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
   src/yei_sensor.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)