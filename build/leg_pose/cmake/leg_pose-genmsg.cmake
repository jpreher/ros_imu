# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "leg_pose: 1 messages, 0 services")

set(MSG_I_FLAGS "-Ileg_pose:/home/debian/ros_imu/src/leg_pose/msg;-Istd_msgs:/home/debian/ros_catkin_ws/install_isolated/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(leg_pose_generate_messages ALL)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(leg_pose
  "/home/debian/ros_imu/src/leg_pose/msg/legPose.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/leg_pose
)

### Generating Services

### Generating Module File
_generate_module_cpp(leg_pose
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/leg_pose
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(leg_pose_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(leg_pose_generate_messages leg_pose_generate_messages_cpp)

# target for backward compatibility
add_custom_target(leg_pose_gencpp)
add_dependencies(leg_pose_gencpp leg_pose_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS leg_pose_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(leg_pose
  "/home/debian/ros_imu/src/leg_pose/msg/legPose.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/leg_pose
)

### Generating Services

### Generating Module File
_generate_module_lisp(leg_pose
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/leg_pose
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(leg_pose_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(leg_pose_generate_messages leg_pose_generate_messages_lisp)

# target for backward compatibility
add_custom_target(leg_pose_genlisp)
add_dependencies(leg_pose_genlisp leg_pose_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS leg_pose_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(leg_pose
  "/home/debian/ros_imu/src/leg_pose/msg/legPose.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/leg_pose
)

### Generating Services

### Generating Module File
_generate_module_py(leg_pose
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/leg_pose
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(leg_pose_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(leg_pose_generate_messages leg_pose_generate_messages_py)

# target for backward compatibility
add_custom_target(leg_pose_genpy)
add_dependencies(leg_pose_genpy leg_pose_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS leg_pose_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/leg_pose)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/leg_pose
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(leg_pose_generate_messages_cpp std_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/leg_pose)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/leg_pose
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(leg_pose_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/leg_pose)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/leg_pose\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/leg_pose
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(leg_pose_generate_messages_py std_msgs_generate_messages_py)
