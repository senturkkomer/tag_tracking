# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "robotic_arm: 2 messages, 0 services")

set(MSG_I_FLAGS "-Irobotic_arm:/home/omer/catkin_ws/src/robotic_arm/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(robotic_arm_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/omer/catkin_ws/src/robotic_arm/msg/joint_position.msg" NAME_WE)
add_custom_target(_robotic_arm_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robotic_arm" "/home/omer/catkin_ws/src/robotic_arm/msg/joint_position.msg" ""
)

get_filename_component(_filename "/home/omer/catkin_ws/src/robotic_arm/msg/joint_position_array.msg" NAME_WE)
add_custom_target(_robotic_arm_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robotic_arm" "/home/omer/catkin_ws/src/robotic_arm/msg/joint_position_array.msg" "robotic_arm/joint_position"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(robotic_arm
  "/home/omer/catkin_ws/src/robotic_arm/msg/joint_position.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robotic_arm
)
_generate_msg_cpp(robotic_arm
  "/home/omer/catkin_ws/src/robotic_arm/msg/joint_position_array.msg"
  "${MSG_I_FLAGS}"
  "/home/omer/catkin_ws/src/robotic_arm/msg/joint_position.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robotic_arm
)

### Generating Services

### Generating Module File
_generate_module_cpp(robotic_arm
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robotic_arm
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(robotic_arm_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(robotic_arm_generate_messages robotic_arm_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/omer/catkin_ws/src/robotic_arm/msg/joint_position.msg" NAME_WE)
add_dependencies(robotic_arm_generate_messages_cpp _robotic_arm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/omer/catkin_ws/src/robotic_arm/msg/joint_position_array.msg" NAME_WE)
add_dependencies(robotic_arm_generate_messages_cpp _robotic_arm_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robotic_arm_gencpp)
add_dependencies(robotic_arm_gencpp robotic_arm_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robotic_arm_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(robotic_arm
  "/home/omer/catkin_ws/src/robotic_arm/msg/joint_position.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robotic_arm
)
_generate_msg_eus(robotic_arm
  "/home/omer/catkin_ws/src/robotic_arm/msg/joint_position_array.msg"
  "${MSG_I_FLAGS}"
  "/home/omer/catkin_ws/src/robotic_arm/msg/joint_position.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robotic_arm
)

### Generating Services

### Generating Module File
_generate_module_eus(robotic_arm
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robotic_arm
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(robotic_arm_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(robotic_arm_generate_messages robotic_arm_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/omer/catkin_ws/src/robotic_arm/msg/joint_position.msg" NAME_WE)
add_dependencies(robotic_arm_generate_messages_eus _robotic_arm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/omer/catkin_ws/src/robotic_arm/msg/joint_position_array.msg" NAME_WE)
add_dependencies(robotic_arm_generate_messages_eus _robotic_arm_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robotic_arm_geneus)
add_dependencies(robotic_arm_geneus robotic_arm_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robotic_arm_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(robotic_arm
  "/home/omer/catkin_ws/src/robotic_arm/msg/joint_position.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robotic_arm
)
_generate_msg_lisp(robotic_arm
  "/home/omer/catkin_ws/src/robotic_arm/msg/joint_position_array.msg"
  "${MSG_I_FLAGS}"
  "/home/omer/catkin_ws/src/robotic_arm/msg/joint_position.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robotic_arm
)

### Generating Services

### Generating Module File
_generate_module_lisp(robotic_arm
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robotic_arm
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(robotic_arm_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(robotic_arm_generate_messages robotic_arm_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/omer/catkin_ws/src/robotic_arm/msg/joint_position.msg" NAME_WE)
add_dependencies(robotic_arm_generate_messages_lisp _robotic_arm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/omer/catkin_ws/src/robotic_arm/msg/joint_position_array.msg" NAME_WE)
add_dependencies(robotic_arm_generate_messages_lisp _robotic_arm_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robotic_arm_genlisp)
add_dependencies(robotic_arm_genlisp robotic_arm_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robotic_arm_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(robotic_arm
  "/home/omer/catkin_ws/src/robotic_arm/msg/joint_position.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robotic_arm
)
_generate_msg_nodejs(robotic_arm
  "/home/omer/catkin_ws/src/robotic_arm/msg/joint_position_array.msg"
  "${MSG_I_FLAGS}"
  "/home/omer/catkin_ws/src/robotic_arm/msg/joint_position.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robotic_arm
)

### Generating Services

### Generating Module File
_generate_module_nodejs(robotic_arm
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robotic_arm
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(robotic_arm_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(robotic_arm_generate_messages robotic_arm_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/omer/catkin_ws/src/robotic_arm/msg/joint_position.msg" NAME_WE)
add_dependencies(robotic_arm_generate_messages_nodejs _robotic_arm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/omer/catkin_ws/src/robotic_arm/msg/joint_position_array.msg" NAME_WE)
add_dependencies(robotic_arm_generate_messages_nodejs _robotic_arm_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robotic_arm_gennodejs)
add_dependencies(robotic_arm_gennodejs robotic_arm_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robotic_arm_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(robotic_arm
  "/home/omer/catkin_ws/src/robotic_arm/msg/joint_position.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robotic_arm
)
_generate_msg_py(robotic_arm
  "/home/omer/catkin_ws/src/robotic_arm/msg/joint_position_array.msg"
  "${MSG_I_FLAGS}"
  "/home/omer/catkin_ws/src/robotic_arm/msg/joint_position.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robotic_arm
)

### Generating Services

### Generating Module File
_generate_module_py(robotic_arm
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robotic_arm
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(robotic_arm_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(robotic_arm_generate_messages robotic_arm_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/omer/catkin_ws/src/robotic_arm/msg/joint_position.msg" NAME_WE)
add_dependencies(robotic_arm_generate_messages_py _robotic_arm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/omer/catkin_ws/src/robotic_arm/msg/joint_position_array.msg" NAME_WE)
add_dependencies(robotic_arm_generate_messages_py _robotic_arm_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robotic_arm_genpy)
add_dependencies(robotic_arm_genpy robotic_arm_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robotic_arm_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robotic_arm)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robotic_arm
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(robotic_arm_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robotic_arm)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robotic_arm
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(robotic_arm_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robotic_arm)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robotic_arm
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(robotic_arm_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robotic_arm)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robotic_arm
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(robotic_arm_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robotic_arm)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robotic_arm\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robotic_arm
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(robotic_arm_generate_messages_py std_msgs_generate_messages_py)
endif()
