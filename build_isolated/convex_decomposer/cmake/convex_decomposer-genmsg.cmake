# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "convex_decomposer: 2 messages, 0 services")

set(MSG_I_FLAGS "-Iconvex_decomposer:/home/sangita/project_ws/AER1516/src/convex_decomposer/msg;-Ishape_msgs:/opt/ros/melodic/share/shape_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(convex_decomposer_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/convex_decomposer/msg/CvxDecomp.msg" NAME_WE)
add_custom_target(_convex_decomposer_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "convex_decomposer" "/home/sangita/project_ws/AER1516/src/convex_decomposer/msg/CvxDecomp.msg" "shape_msgs/Plane:convex_decomposer/Polyhedron:std_msgs/Header"
)

get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/convex_decomposer/msg/Polyhedron.msg" NAME_WE)
add_custom_target(_convex_decomposer_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "convex_decomposer" "/home/sangita/project_ws/AER1516/src/convex_decomposer/msg/Polyhedron.msg" "shape_msgs/Plane"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(convex_decomposer
  "/home/sangita/project_ws/AER1516/src/convex_decomposer/msg/CvxDecomp.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/shape_msgs/cmake/../msg/Plane.msg;/home/sangita/project_ws/AER1516/src/convex_decomposer/msg/Polyhedron.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/convex_decomposer
)
_generate_msg_cpp(convex_decomposer
  "/home/sangita/project_ws/AER1516/src/convex_decomposer/msg/Polyhedron.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/shape_msgs/cmake/../msg/Plane.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/convex_decomposer
)

### Generating Services

### Generating Module File
_generate_module_cpp(convex_decomposer
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/convex_decomposer
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(convex_decomposer_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(convex_decomposer_generate_messages convex_decomposer_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/convex_decomposer/msg/CvxDecomp.msg" NAME_WE)
add_dependencies(convex_decomposer_generate_messages_cpp _convex_decomposer_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/convex_decomposer/msg/Polyhedron.msg" NAME_WE)
add_dependencies(convex_decomposer_generate_messages_cpp _convex_decomposer_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(convex_decomposer_gencpp)
add_dependencies(convex_decomposer_gencpp convex_decomposer_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS convex_decomposer_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(convex_decomposer
  "/home/sangita/project_ws/AER1516/src/convex_decomposer/msg/CvxDecomp.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/shape_msgs/cmake/../msg/Plane.msg;/home/sangita/project_ws/AER1516/src/convex_decomposer/msg/Polyhedron.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/convex_decomposer
)
_generate_msg_eus(convex_decomposer
  "/home/sangita/project_ws/AER1516/src/convex_decomposer/msg/Polyhedron.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/shape_msgs/cmake/../msg/Plane.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/convex_decomposer
)

### Generating Services

### Generating Module File
_generate_module_eus(convex_decomposer
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/convex_decomposer
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(convex_decomposer_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(convex_decomposer_generate_messages convex_decomposer_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/convex_decomposer/msg/CvxDecomp.msg" NAME_WE)
add_dependencies(convex_decomposer_generate_messages_eus _convex_decomposer_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/convex_decomposer/msg/Polyhedron.msg" NAME_WE)
add_dependencies(convex_decomposer_generate_messages_eus _convex_decomposer_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(convex_decomposer_geneus)
add_dependencies(convex_decomposer_geneus convex_decomposer_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS convex_decomposer_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(convex_decomposer
  "/home/sangita/project_ws/AER1516/src/convex_decomposer/msg/CvxDecomp.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/shape_msgs/cmake/../msg/Plane.msg;/home/sangita/project_ws/AER1516/src/convex_decomposer/msg/Polyhedron.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/convex_decomposer
)
_generate_msg_lisp(convex_decomposer
  "/home/sangita/project_ws/AER1516/src/convex_decomposer/msg/Polyhedron.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/shape_msgs/cmake/../msg/Plane.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/convex_decomposer
)

### Generating Services

### Generating Module File
_generate_module_lisp(convex_decomposer
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/convex_decomposer
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(convex_decomposer_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(convex_decomposer_generate_messages convex_decomposer_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/convex_decomposer/msg/CvxDecomp.msg" NAME_WE)
add_dependencies(convex_decomposer_generate_messages_lisp _convex_decomposer_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/convex_decomposer/msg/Polyhedron.msg" NAME_WE)
add_dependencies(convex_decomposer_generate_messages_lisp _convex_decomposer_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(convex_decomposer_genlisp)
add_dependencies(convex_decomposer_genlisp convex_decomposer_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS convex_decomposer_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(convex_decomposer
  "/home/sangita/project_ws/AER1516/src/convex_decomposer/msg/CvxDecomp.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/shape_msgs/cmake/../msg/Plane.msg;/home/sangita/project_ws/AER1516/src/convex_decomposer/msg/Polyhedron.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/convex_decomposer
)
_generate_msg_nodejs(convex_decomposer
  "/home/sangita/project_ws/AER1516/src/convex_decomposer/msg/Polyhedron.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/shape_msgs/cmake/../msg/Plane.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/convex_decomposer
)

### Generating Services

### Generating Module File
_generate_module_nodejs(convex_decomposer
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/convex_decomposer
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(convex_decomposer_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(convex_decomposer_generate_messages convex_decomposer_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/convex_decomposer/msg/CvxDecomp.msg" NAME_WE)
add_dependencies(convex_decomposer_generate_messages_nodejs _convex_decomposer_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/convex_decomposer/msg/Polyhedron.msg" NAME_WE)
add_dependencies(convex_decomposer_generate_messages_nodejs _convex_decomposer_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(convex_decomposer_gennodejs)
add_dependencies(convex_decomposer_gennodejs convex_decomposer_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS convex_decomposer_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(convex_decomposer
  "/home/sangita/project_ws/AER1516/src/convex_decomposer/msg/CvxDecomp.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/shape_msgs/cmake/../msg/Plane.msg;/home/sangita/project_ws/AER1516/src/convex_decomposer/msg/Polyhedron.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/convex_decomposer
)
_generate_msg_py(convex_decomposer
  "/home/sangita/project_ws/AER1516/src/convex_decomposer/msg/Polyhedron.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/shape_msgs/cmake/../msg/Plane.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/convex_decomposer
)

### Generating Services

### Generating Module File
_generate_module_py(convex_decomposer
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/convex_decomposer
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(convex_decomposer_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(convex_decomposer_generate_messages convex_decomposer_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/convex_decomposer/msg/CvxDecomp.msg" NAME_WE)
add_dependencies(convex_decomposer_generate_messages_py _convex_decomposer_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/convex_decomposer/msg/Polyhedron.msg" NAME_WE)
add_dependencies(convex_decomposer_generate_messages_py _convex_decomposer_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(convex_decomposer_genpy)
add_dependencies(convex_decomposer_genpy convex_decomposer_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS convex_decomposer_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/convex_decomposer)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/convex_decomposer
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET shape_msgs_generate_messages_cpp)
  add_dependencies(convex_decomposer_generate_messages_cpp shape_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/convex_decomposer)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/convex_decomposer
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET shape_msgs_generate_messages_eus)
  add_dependencies(convex_decomposer_generate_messages_eus shape_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/convex_decomposer)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/convex_decomposer
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET shape_msgs_generate_messages_lisp)
  add_dependencies(convex_decomposer_generate_messages_lisp shape_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/convex_decomposer)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/convex_decomposer
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET shape_msgs_generate_messages_nodejs)
  add_dependencies(convex_decomposer_generate_messages_nodejs shape_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/convex_decomposer)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/convex_decomposer\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/convex_decomposer
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET shape_msgs_generate_messages_py)
  add_dependencies(convex_decomposer_generate_messages_py shape_msgs_generate_messages_py)
endif()
