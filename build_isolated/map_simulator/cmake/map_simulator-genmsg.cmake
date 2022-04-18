# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "map_simulator: 1 messages, 0 services")

set(MSG_I_FLAGS "-Imap_simulator:/home/sangita/project_ws/AER1516/src/map_simulator/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(map_simulator_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/map_simulator/msg/Map3D.msg" NAME_WE)
add_custom_target(_map_simulator_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "map_simulator" "/home/sangita/project_ws/AER1516/src/map_simulator/msg/Map3D.msg" "geometry_msgs/Vector3"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(map_simulator
  "/home/sangita/project_ws/AER1516/src/map_simulator/msg/Map3D.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/map_simulator
)

### Generating Services

### Generating Module File
_generate_module_cpp(map_simulator
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/map_simulator
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(map_simulator_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(map_simulator_generate_messages map_simulator_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/map_simulator/msg/Map3D.msg" NAME_WE)
add_dependencies(map_simulator_generate_messages_cpp _map_simulator_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(map_simulator_gencpp)
add_dependencies(map_simulator_gencpp map_simulator_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS map_simulator_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(map_simulator
  "/home/sangita/project_ws/AER1516/src/map_simulator/msg/Map3D.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/map_simulator
)

### Generating Services

### Generating Module File
_generate_module_eus(map_simulator
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/map_simulator
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(map_simulator_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(map_simulator_generate_messages map_simulator_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/map_simulator/msg/Map3D.msg" NAME_WE)
add_dependencies(map_simulator_generate_messages_eus _map_simulator_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(map_simulator_geneus)
add_dependencies(map_simulator_geneus map_simulator_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS map_simulator_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(map_simulator
  "/home/sangita/project_ws/AER1516/src/map_simulator/msg/Map3D.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/map_simulator
)

### Generating Services

### Generating Module File
_generate_module_lisp(map_simulator
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/map_simulator
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(map_simulator_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(map_simulator_generate_messages map_simulator_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/map_simulator/msg/Map3D.msg" NAME_WE)
add_dependencies(map_simulator_generate_messages_lisp _map_simulator_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(map_simulator_genlisp)
add_dependencies(map_simulator_genlisp map_simulator_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS map_simulator_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(map_simulator
  "/home/sangita/project_ws/AER1516/src/map_simulator/msg/Map3D.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/map_simulator
)

### Generating Services

### Generating Module File
_generate_module_nodejs(map_simulator
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/map_simulator
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(map_simulator_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(map_simulator_generate_messages map_simulator_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/map_simulator/msg/Map3D.msg" NAME_WE)
add_dependencies(map_simulator_generate_messages_nodejs _map_simulator_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(map_simulator_gennodejs)
add_dependencies(map_simulator_gennodejs map_simulator_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS map_simulator_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(map_simulator
  "/home/sangita/project_ws/AER1516/src/map_simulator/msg/Map3D.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/map_simulator
)

### Generating Services

### Generating Module File
_generate_module_py(map_simulator
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/map_simulator
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(map_simulator_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(map_simulator_generate_messages map_simulator_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/map_simulator/msg/Map3D.msg" NAME_WE)
add_dependencies(map_simulator_generate_messages_py _map_simulator_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(map_simulator_genpy)
add_dependencies(map_simulator_genpy map_simulator_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS map_simulator_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/map_simulator)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/map_simulator
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(map_simulator_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(map_simulator_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/map_simulator)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/map_simulator
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(map_simulator_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(map_simulator_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/map_simulator)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/map_simulator
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(map_simulator_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(map_simulator_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/map_simulator)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/map_simulator
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(map_simulator_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(map_simulator_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/map_simulator)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/map_simulator\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/map_simulator
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(map_simulator_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(map_simulator_generate_messages_py geometry_msgs_generate_messages_py)
endif()
