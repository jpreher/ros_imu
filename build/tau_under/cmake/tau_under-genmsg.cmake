# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "tau_under: 1 messages, 0 services")

set(MSG_I_FLAGS "-Itau_under:/home/aames/ros_imu/src/tau_under/msg;-Istd_msgs:/opt/ros/hydro/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(tau_under_generate_messages ALL)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(tau_under
  "/home/aames/ros_imu/src/tau_under/msg/tau_under_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tau_under
)

### Generating Services

### Generating Module File
_generate_module_cpp(tau_under
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tau_under
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(tau_under_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(tau_under_generate_messages tau_under_generate_messages_cpp)

# target for backward compatibility
add_custom_target(tau_under_gencpp)
add_dependencies(tau_under_gencpp tau_under_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tau_under_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(tau_under
  "/home/aames/ros_imu/src/tau_under/msg/tau_under_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tau_under
)

### Generating Services

### Generating Module File
_generate_module_lisp(tau_under
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tau_under
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(tau_under_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(tau_under_generate_messages tau_under_generate_messages_lisp)

# target for backward compatibility
add_custom_target(tau_under_genlisp)
add_dependencies(tau_under_genlisp tau_under_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tau_under_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(tau_under
  "/home/aames/ros_imu/src/tau_under/msg/tau_under_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tau_under
)

### Generating Services

### Generating Module File
_generate_module_py(tau_under
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tau_under
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(tau_under_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(tau_under_generate_messages tau_under_generate_messages_py)

# target for backward compatibility
add_custom_target(tau_under_genpy)
add_dependencies(tau_under_genpy tau_under_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tau_under_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tau_under)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tau_under
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(tau_under_generate_messages_cpp std_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tau_under)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tau_under
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(tau_under_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tau_under)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tau_under\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tau_under
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(tau_under_generate_messages_py std_msgs_generate_messages_py)
