# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "srl_final: 1 messages, 0 services")

set(MSG_I_FLAGS "-Isrl_final:/home/harlab/ros_ws_1/src/srl_final/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(srl_final_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/harlab/ros_ws_1/src/srl_final/msg/glove.msg" NAME_WE)
add_custom_target(_srl_final_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "srl_final" "/home/harlab/ros_ws_1/src/srl_final/msg/glove.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(srl_final
  "/home/harlab/ros_ws_1/src/srl_final/msg/glove.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/srl_final
)

### Generating Services

### Generating Module File
_generate_module_cpp(srl_final
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/srl_final
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(srl_final_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(srl_final_generate_messages srl_final_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/harlab/ros_ws_1/src/srl_final/msg/glove.msg" NAME_WE)
add_dependencies(srl_final_generate_messages_cpp _srl_final_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(srl_final_gencpp)
add_dependencies(srl_final_gencpp srl_final_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS srl_final_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(srl_final
  "/home/harlab/ros_ws_1/src/srl_final/msg/glove.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/srl_final
)

### Generating Services

### Generating Module File
_generate_module_eus(srl_final
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/srl_final
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(srl_final_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(srl_final_generate_messages srl_final_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/harlab/ros_ws_1/src/srl_final/msg/glove.msg" NAME_WE)
add_dependencies(srl_final_generate_messages_eus _srl_final_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(srl_final_geneus)
add_dependencies(srl_final_geneus srl_final_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS srl_final_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(srl_final
  "/home/harlab/ros_ws_1/src/srl_final/msg/glove.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/srl_final
)

### Generating Services

### Generating Module File
_generate_module_lisp(srl_final
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/srl_final
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(srl_final_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(srl_final_generate_messages srl_final_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/harlab/ros_ws_1/src/srl_final/msg/glove.msg" NAME_WE)
add_dependencies(srl_final_generate_messages_lisp _srl_final_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(srl_final_genlisp)
add_dependencies(srl_final_genlisp srl_final_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS srl_final_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(srl_final
  "/home/harlab/ros_ws_1/src/srl_final/msg/glove.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/srl_final
)

### Generating Services

### Generating Module File
_generate_module_nodejs(srl_final
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/srl_final
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(srl_final_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(srl_final_generate_messages srl_final_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/harlab/ros_ws_1/src/srl_final/msg/glove.msg" NAME_WE)
add_dependencies(srl_final_generate_messages_nodejs _srl_final_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(srl_final_gennodejs)
add_dependencies(srl_final_gennodejs srl_final_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS srl_final_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(srl_final
  "/home/harlab/ros_ws_1/src/srl_final/msg/glove.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/srl_final
)

### Generating Services

### Generating Module File
_generate_module_py(srl_final
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/srl_final
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(srl_final_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(srl_final_generate_messages srl_final_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/harlab/ros_ws_1/src/srl_final/msg/glove.msg" NAME_WE)
add_dependencies(srl_final_generate_messages_py _srl_final_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(srl_final_genpy)
add_dependencies(srl_final_genpy srl_final_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS srl_final_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/srl_final)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/srl_final
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(srl_final_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/srl_final)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/srl_final
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(srl_final_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/srl_final)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/srl_final
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(srl_final_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/srl_final)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/srl_final
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(srl_final_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/srl_final)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/srl_final\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/srl_final
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(srl_final_generate_messages_py std_msgs_generate_messages_py)
endif()
