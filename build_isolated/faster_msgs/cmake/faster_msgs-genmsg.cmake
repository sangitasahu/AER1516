# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "faster_msgs: 1 messages, 0 services")

set(MSG_I_FLAGS "-Ifaster_msgs:/home/sangita/project_ws/AER1516/src/faster/faster_msgs/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(faster_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/faster/faster_msgs/msg/Mode.msg" NAME_WE)
add_custom_target(_faster_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "faster_msgs" "/home/sangita/project_ws/AER1516/src/faster/faster_msgs/msg/Mode.msg" "std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(faster_msgs
  "/home/sangita/project_ws/AER1516/src/faster/faster_msgs/msg/Mode.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/faster_msgs
)

### Generating Services

### Generating Module File
_generate_module_cpp(faster_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/faster_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(faster_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(faster_msgs_generate_messages faster_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/faster/faster_msgs/msg/Mode.msg" NAME_WE)
add_dependencies(faster_msgs_generate_messages_cpp _faster_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(faster_msgs_gencpp)
add_dependencies(faster_msgs_gencpp faster_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS faster_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(faster_msgs
  "/home/sangita/project_ws/AER1516/src/faster/faster_msgs/msg/Mode.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/faster_msgs
)

### Generating Services

### Generating Module File
_generate_module_eus(faster_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/faster_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(faster_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(faster_msgs_generate_messages faster_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/faster/faster_msgs/msg/Mode.msg" NAME_WE)
add_dependencies(faster_msgs_generate_messages_eus _faster_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(faster_msgs_geneus)
add_dependencies(faster_msgs_geneus faster_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS faster_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(faster_msgs
  "/home/sangita/project_ws/AER1516/src/faster/faster_msgs/msg/Mode.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/faster_msgs
)

### Generating Services

### Generating Module File
_generate_module_lisp(faster_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/faster_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(faster_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(faster_msgs_generate_messages faster_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/faster/faster_msgs/msg/Mode.msg" NAME_WE)
add_dependencies(faster_msgs_generate_messages_lisp _faster_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(faster_msgs_genlisp)
add_dependencies(faster_msgs_genlisp faster_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS faster_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(faster_msgs
  "/home/sangita/project_ws/AER1516/src/faster/faster_msgs/msg/Mode.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/faster_msgs
)

### Generating Services

### Generating Module File
_generate_module_nodejs(faster_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/faster_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(faster_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(faster_msgs_generate_messages faster_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/faster/faster_msgs/msg/Mode.msg" NAME_WE)
add_dependencies(faster_msgs_generate_messages_nodejs _faster_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(faster_msgs_gennodejs)
add_dependencies(faster_msgs_gennodejs faster_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS faster_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(faster_msgs
  "/home/sangita/project_ws/AER1516/src/faster/faster_msgs/msg/Mode.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/faster_msgs
)

### Generating Services

### Generating Module File
_generate_module_py(faster_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/faster_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(faster_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(faster_msgs_generate_messages faster_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/faster/faster_msgs/msg/Mode.msg" NAME_WE)
add_dependencies(faster_msgs_generate_messages_py _faster_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(faster_msgs_genpy)
add_dependencies(faster_msgs_genpy faster_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS faster_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/faster_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/faster_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(faster_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(faster_msgs_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/faster_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/faster_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(faster_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(faster_msgs_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/faster_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/faster_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(faster_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(faster_msgs_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/faster_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/faster_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(faster_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(faster_msgs_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/faster_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/faster_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/faster_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(faster_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(faster_msgs_generate_messages_py geometry_msgs_generate_messages_py)
endif()
