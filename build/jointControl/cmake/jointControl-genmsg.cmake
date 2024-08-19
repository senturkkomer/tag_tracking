# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "jointControl: 0 messages, 1 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(jointControl_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/omer/catkin_ws/src/jointControl/srv/RobotMove.srv" NAME_WE)
add_custom_target(_jointControl_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "jointControl" "/home/omer/catkin_ws/src/jointControl/srv/RobotMove.srv" "geometry_msgs/Pose:geometry_msgs/Point:geometry_msgs/Quaternion"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(jointControl
  "/home/omer/catkin_ws/src/jointControl/srv/RobotMove.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/jointControl
)

### Generating Module File
_generate_module_cpp(jointControl
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/jointControl
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(jointControl_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(jointControl_generate_messages jointControl_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/omer/catkin_ws/src/jointControl/srv/RobotMove.srv" NAME_WE)
add_dependencies(jointControl_generate_messages_cpp _jointControl_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(jointControl_gencpp)
add_dependencies(jointControl_gencpp jointControl_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS jointControl_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(jointControl
  "/home/omer/catkin_ws/src/jointControl/srv/RobotMove.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/jointControl
)

### Generating Module File
_generate_module_eus(jointControl
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/jointControl
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(jointControl_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(jointControl_generate_messages jointControl_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/omer/catkin_ws/src/jointControl/srv/RobotMove.srv" NAME_WE)
add_dependencies(jointControl_generate_messages_eus _jointControl_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(jointControl_geneus)
add_dependencies(jointControl_geneus jointControl_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS jointControl_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(jointControl
  "/home/omer/catkin_ws/src/jointControl/srv/RobotMove.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/jointControl
)

### Generating Module File
_generate_module_lisp(jointControl
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/jointControl
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(jointControl_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(jointControl_generate_messages jointControl_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/omer/catkin_ws/src/jointControl/srv/RobotMove.srv" NAME_WE)
add_dependencies(jointControl_generate_messages_lisp _jointControl_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(jointControl_genlisp)
add_dependencies(jointControl_genlisp jointControl_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS jointControl_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(jointControl
  "/home/omer/catkin_ws/src/jointControl/srv/RobotMove.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/jointControl
)

### Generating Module File
_generate_module_nodejs(jointControl
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/jointControl
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(jointControl_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(jointControl_generate_messages jointControl_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/omer/catkin_ws/src/jointControl/srv/RobotMove.srv" NAME_WE)
add_dependencies(jointControl_generate_messages_nodejs _jointControl_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(jointControl_gennodejs)
add_dependencies(jointControl_gennodejs jointControl_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS jointControl_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(jointControl
  "/home/omer/catkin_ws/src/jointControl/srv/RobotMove.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/jointControl
)

### Generating Module File
_generate_module_py(jointControl
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/jointControl
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(jointControl_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(jointControl_generate_messages jointControl_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/omer/catkin_ws/src/jointControl/srv/RobotMove.srv" NAME_WE)
add_dependencies(jointControl_generate_messages_py _jointControl_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(jointControl_genpy)
add_dependencies(jointControl_genpy jointControl_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS jointControl_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/jointControl)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/jointControl
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(jointControl_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(jointControl_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/jointControl)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/jointControl
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(jointControl_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(jointControl_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/jointControl)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/jointControl
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(jointControl_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(jointControl_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/jointControl)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/jointControl
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(jointControl_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(jointControl_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/jointControl)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/jointControl\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/jointControl
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(jointControl_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(jointControl_generate_messages_py geometry_msgs_generate_messages_py)
endif()
