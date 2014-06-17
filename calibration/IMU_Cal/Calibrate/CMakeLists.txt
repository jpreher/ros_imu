project(BBB_MPU9150_Calibrate)
cmake_minimum_required(VERSION 2.8)

add_definitions(-std=c++0x)

aux_source_directory(. SRC_LIST)
add_executable(${PROJECT_NAME} ${SRC_LIST})

