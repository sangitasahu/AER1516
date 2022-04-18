# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "fla_msgs: 14 messages, 0 services")

set(MSG_I_FLAGS "-Ifla_msgs:/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Inav_msgs:/opt/ros/melodic/share/nav_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/melodic/share/sensor_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/melodic/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(fla_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/Latency.msg" NAME_WE)
add_custom_target(_fla_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fla_msgs" "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/Latency.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/ImageDetections.msg" NAME_WE)
add_custom_target(_fla_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fla_msgs" "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/ImageDetections.msg" "fla_msgs/Detection:std_msgs/Header"
)

get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/FlightEvent.msg" NAME_WE)
add_custom_target(_fla_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fla_msgs" "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/FlightEvent.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/NodeStatus.msg" NAME_WE)
add_custom_target(_fla_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fla_msgs" "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/NodeStatus.msg" ""
)

get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/ControlMessage.msg" NAME_WE)
add_custom_target(_fla_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fla_msgs" "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/ControlMessage.msg" ""
)

get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/NodeList.msg" NAME_WE)
add_custom_target(_fla_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fla_msgs" "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/NodeList.msg" "fla_msgs/NodeStatus"
)

get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/ProcessStatus.msg" NAME_WE)
add_custom_target(_fla_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fla_msgs" "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/ProcessStatus.msg" ""
)

get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/TelemString.msg" NAME_WE)
add_custom_target(_fla_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fla_msgs" "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/TelemString.msg" ""
)

get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/FlightCommand.msg" NAME_WE)
add_custom_target(_fla_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fla_msgs" "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/FlightCommand.msg" ""
)

get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/Detection.msg" NAME_WE)
add_custom_target(_fla_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fla_msgs" "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/Detection.msg" ""
)

get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/FlightStateTransition.msg" NAME_WE)
add_custom_target(_fla_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fla_msgs" "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/FlightStateTransition.msg" "std_msgs/String:std_msgs/Header"
)

get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/WaypointList.msg" NAME_WE)
add_custom_target(_fla_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fla_msgs" "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/WaypointList.msg" "geometry_msgs/Pose:std_msgs/Header:std_msgs/Float64:geometry_msgs/Point:geometry_msgs/PoseStamped:geometry_msgs/Quaternion"
)

get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/JoyDef.msg" NAME_WE)
add_custom_target(_fla_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fla_msgs" "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/JoyDef.msg" ""
)

get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/Box.msg" NAME_WE)
add_custom_target(_fla_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fla_msgs" "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/Box.msg" "geometry_msgs/Vector3:std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(fla_msgs
  "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/Latency.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fla_msgs
)
_generate_msg_cpp(fla_msgs
  "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/ImageDetections.msg"
  "${MSG_I_FLAGS}"
  "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/Detection.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fla_msgs
)
_generate_msg_cpp(fla_msgs
  "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/FlightEvent.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fla_msgs
)
_generate_msg_cpp(fla_msgs
  "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/NodeStatus.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fla_msgs
)
_generate_msg_cpp(fla_msgs
  "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/ControlMessage.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fla_msgs
)
_generate_msg_cpp(fla_msgs
  "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/NodeList.msg"
  "${MSG_I_FLAGS}"
  "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/NodeStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fla_msgs
)
_generate_msg_cpp(fla_msgs
  "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/TelemString.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fla_msgs
)
_generate_msg_cpp(fla_msgs
  "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/FlightCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fla_msgs
)
_generate_msg_cpp(fla_msgs
  "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/Box.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fla_msgs
)
_generate_msg_cpp(fla_msgs
  "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/FlightStateTransition.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fla_msgs
)
_generate_msg_cpp(fla_msgs
  "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/WaypointList.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Float64.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fla_msgs
)
_generate_msg_cpp(fla_msgs
  "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/Detection.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fla_msgs
)
_generate_msg_cpp(fla_msgs
  "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/JoyDef.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fla_msgs
)
_generate_msg_cpp(fla_msgs
  "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/ProcessStatus.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fla_msgs
)

### Generating Services

### Generating Module File
_generate_module_cpp(fla_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fla_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(fla_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(fla_msgs_generate_messages fla_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/Latency.msg" NAME_WE)
add_dependencies(fla_msgs_generate_messages_cpp _fla_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/ImageDetections.msg" NAME_WE)
add_dependencies(fla_msgs_generate_messages_cpp _fla_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/FlightEvent.msg" NAME_WE)
add_dependencies(fla_msgs_generate_messages_cpp _fla_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/NodeStatus.msg" NAME_WE)
add_dependencies(fla_msgs_generate_messages_cpp _fla_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/ControlMessage.msg" NAME_WE)
add_dependencies(fla_msgs_generate_messages_cpp _fla_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/NodeList.msg" NAME_WE)
add_dependencies(fla_msgs_generate_messages_cpp _fla_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/ProcessStatus.msg" NAME_WE)
add_dependencies(fla_msgs_generate_messages_cpp _fla_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/TelemString.msg" NAME_WE)
add_dependencies(fla_msgs_generate_messages_cpp _fla_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/FlightCommand.msg" NAME_WE)
add_dependencies(fla_msgs_generate_messages_cpp _fla_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/Detection.msg" NAME_WE)
add_dependencies(fla_msgs_generate_messages_cpp _fla_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/FlightStateTransition.msg" NAME_WE)
add_dependencies(fla_msgs_generate_messages_cpp _fla_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/WaypointList.msg" NAME_WE)
add_dependencies(fla_msgs_generate_messages_cpp _fla_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/JoyDef.msg" NAME_WE)
add_dependencies(fla_msgs_generate_messages_cpp _fla_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/Box.msg" NAME_WE)
add_dependencies(fla_msgs_generate_messages_cpp _fla_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(fla_msgs_gencpp)
add_dependencies(fla_msgs_gencpp fla_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS fla_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(fla_msgs
  "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/Latency.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fla_msgs
)
_generate_msg_eus(fla_msgs
  "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/ImageDetections.msg"
  "${MSG_I_FLAGS}"
  "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/Detection.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fla_msgs
)
_generate_msg_eus(fla_msgs
  "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/FlightEvent.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fla_msgs
)
_generate_msg_eus(fla_msgs
  "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/NodeStatus.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fla_msgs
)
_generate_msg_eus(fla_msgs
  "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/ControlMessage.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fla_msgs
)
_generate_msg_eus(fla_msgs
  "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/NodeList.msg"
  "${MSG_I_FLAGS}"
  "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/NodeStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fla_msgs
)
_generate_msg_eus(fla_msgs
  "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/TelemString.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fla_msgs
)
_generate_msg_eus(fla_msgs
  "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/FlightCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fla_msgs
)
_generate_msg_eus(fla_msgs
  "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/Box.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fla_msgs
)
_generate_msg_eus(fla_msgs
  "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/FlightStateTransition.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fla_msgs
)
_generate_msg_eus(fla_msgs
  "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/WaypointList.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Float64.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fla_msgs
)
_generate_msg_eus(fla_msgs
  "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/Detection.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fla_msgs
)
_generate_msg_eus(fla_msgs
  "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/JoyDef.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fla_msgs
)
_generate_msg_eus(fla_msgs
  "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/ProcessStatus.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fla_msgs
)

### Generating Services

### Generating Module File
_generate_module_eus(fla_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fla_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(fla_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(fla_msgs_generate_messages fla_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/Latency.msg" NAME_WE)
add_dependencies(fla_msgs_generate_messages_eus _fla_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/ImageDetections.msg" NAME_WE)
add_dependencies(fla_msgs_generate_messages_eus _fla_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/FlightEvent.msg" NAME_WE)
add_dependencies(fla_msgs_generate_messages_eus _fla_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/NodeStatus.msg" NAME_WE)
add_dependencies(fla_msgs_generate_messages_eus _fla_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/ControlMessage.msg" NAME_WE)
add_dependencies(fla_msgs_generate_messages_eus _fla_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/NodeList.msg" NAME_WE)
add_dependencies(fla_msgs_generate_messages_eus _fla_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/ProcessStatus.msg" NAME_WE)
add_dependencies(fla_msgs_generate_messages_eus _fla_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/TelemString.msg" NAME_WE)
add_dependencies(fla_msgs_generate_messages_eus _fla_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/FlightCommand.msg" NAME_WE)
add_dependencies(fla_msgs_generate_messages_eus _fla_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/Detection.msg" NAME_WE)
add_dependencies(fla_msgs_generate_messages_eus _fla_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/FlightStateTransition.msg" NAME_WE)
add_dependencies(fla_msgs_generate_messages_eus _fla_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/WaypointList.msg" NAME_WE)
add_dependencies(fla_msgs_generate_messages_eus _fla_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/JoyDef.msg" NAME_WE)
add_dependencies(fla_msgs_generate_messages_eus _fla_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/Box.msg" NAME_WE)
add_dependencies(fla_msgs_generate_messages_eus _fla_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(fla_msgs_geneus)
add_dependencies(fla_msgs_geneus fla_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS fla_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(fla_msgs
  "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/Latency.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fla_msgs
)
_generate_msg_lisp(fla_msgs
  "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/ImageDetections.msg"
  "${MSG_I_FLAGS}"
  "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/Detection.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fla_msgs
)
_generate_msg_lisp(fla_msgs
  "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/FlightEvent.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fla_msgs
)
_generate_msg_lisp(fla_msgs
  "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/NodeStatus.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fla_msgs
)
_generate_msg_lisp(fla_msgs
  "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/ControlMessage.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fla_msgs
)
_generate_msg_lisp(fla_msgs
  "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/NodeList.msg"
  "${MSG_I_FLAGS}"
  "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/NodeStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fla_msgs
)
_generate_msg_lisp(fla_msgs
  "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/TelemString.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fla_msgs
)
_generate_msg_lisp(fla_msgs
  "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/FlightCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fla_msgs
)
_generate_msg_lisp(fla_msgs
  "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/Box.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fla_msgs
)
_generate_msg_lisp(fla_msgs
  "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/FlightStateTransition.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fla_msgs
)
_generate_msg_lisp(fla_msgs
  "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/WaypointList.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Float64.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fla_msgs
)
_generate_msg_lisp(fla_msgs
  "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/Detection.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fla_msgs
)
_generate_msg_lisp(fla_msgs
  "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/JoyDef.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fla_msgs
)
_generate_msg_lisp(fla_msgs
  "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/ProcessStatus.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fla_msgs
)

### Generating Services

### Generating Module File
_generate_module_lisp(fla_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fla_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(fla_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(fla_msgs_generate_messages fla_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/Latency.msg" NAME_WE)
add_dependencies(fla_msgs_generate_messages_lisp _fla_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/ImageDetections.msg" NAME_WE)
add_dependencies(fla_msgs_generate_messages_lisp _fla_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/FlightEvent.msg" NAME_WE)
add_dependencies(fla_msgs_generate_messages_lisp _fla_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/NodeStatus.msg" NAME_WE)
add_dependencies(fla_msgs_generate_messages_lisp _fla_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/ControlMessage.msg" NAME_WE)
add_dependencies(fla_msgs_generate_messages_lisp _fla_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/NodeList.msg" NAME_WE)
add_dependencies(fla_msgs_generate_messages_lisp _fla_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/ProcessStatus.msg" NAME_WE)
add_dependencies(fla_msgs_generate_messages_lisp _fla_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/TelemString.msg" NAME_WE)
add_dependencies(fla_msgs_generate_messages_lisp _fla_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/FlightCommand.msg" NAME_WE)
add_dependencies(fla_msgs_generate_messages_lisp _fla_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/Detection.msg" NAME_WE)
add_dependencies(fla_msgs_generate_messages_lisp _fla_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/FlightStateTransition.msg" NAME_WE)
add_dependencies(fla_msgs_generate_messages_lisp _fla_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/WaypointList.msg" NAME_WE)
add_dependencies(fla_msgs_generate_messages_lisp _fla_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/JoyDef.msg" NAME_WE)
add_dependencies(fla_msgs_generate_messages_lisp _fla_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/Box.msg" NAME_WE)
add_dependencies(fla_msgs_generate_messages_lisp _fla_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(fla_msgs_genlisp)
add_dependencies(fla_msgs_genlisp fla_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS fla_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(fla_msgs
  "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/Latency.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fla_msgs
)
_generate_msg_nodejs(fla_msgs
  "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/ImageDetections.msg"
  "${MSG_I_FLAGS}"
  "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/Detection.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fla_msgs
)
_generate_msg_nodejs(fla_msgs
  "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/FlightEvent.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fla_msgs
)
_generate_msg_nodejs(fla_msgs
  "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/NodeStatus.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fla_msgs
)
_generate_msg_nodejs(fla_msgs
  "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/ControlMessage.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fla_msgs
)
_generate_msg_nodejs(fla_msgs
  "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/NodeList.msg"
  "${MSG_I_FLAGS}"
  "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/NodeStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fla_msgs
)
_generate_msg_nodejs(fla_msgs
  "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/TelemString.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fla_msgs
)
_generate_msg_nodejs(fla_msgs
  "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/FlightCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fla_msgs
)
_generate_msg_nodejs(fla_msgs
  "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/Box.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fla_msgs
)
_generate_msg_nodejs(fla_msgs
  "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/FlightStateTransition.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fla_msgs
)
_generate_msg_nodejs(fla_msgs
  "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/WaypointList.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Float64.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fla_msgs
)
_generate_msg_nodejs(fla_msgs
  "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/Detection.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fla_msgs
)
_generate_msg_nodejs(fla_msgs
  "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/JoyDef.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fla_msgs
)
_generate_msg_nodejs(fla_msgs
  "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/ProcessStatus.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fla_msgs
)

### Generating Services

### Generating Module File
_generate_module_nodejs(fla_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fla_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(fla_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(fla_msgs_generate_messages fla_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/Latency.msg" NAME_WE)
add_dependencies(fla_msgs_generate_messages_nodejs _fla_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/ImageDetections.msg" NAME_WE)
add_dependencies(fla_msgs_generate_messages_nodejs _fla_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/FlightEvent.msg" NAME_WE)
add_dependencies(fla_msgs_generate_messages_nodejs _fla_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/NodeStatus.msg" NAME_WE)
add_dependencies(fla_msgs_generate_messages_nodejs _fla_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/ControlMessage.msg" NAME_WE)
add_dependencies(fla_msgs_generate_messages_nodejs _fla_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/NodeList.msg" NAME_WE)
add_dependencies(fla_msgs_generate_messages_nodejs _fla_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/ProcessStatus.msg" NAME_WE)
add_dependencies(fla_msgs_generate_messages_nodejs _fla_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/TelemString.msg" NAME_WE)
add_dependencies(fla_msgs_generate_messages_nodejs _fla_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/FlightCommand.msg" NAME_WE)
add_dependencies(fla_msgs_generate_messages_nodejs _fla_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/Detection.msg" NAME_WE)
add_dependencies(fla_msgs_generate_messages_nodejs _fla_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/FlightStateTransition.msg" NAME_WE)
add_dependencies(fla_msgs_generate_messages_nodejs _fla_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/WaypointList.msg" NAME_WE)
add_dependencies(fla_msgs_generate_messages_nodejs _fla_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/JoyDef.msg" NAME_WE)
add_dependencies(fla_msgs_generate_messages_nodejs _fla_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/Box.msg" NAME_WE)
add_dependencies(fla_msgs_generate_messages_nodejs _fla_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(fla_msgs_gennodejs)
add_dependencies(fla_msgs_gennodejs fla_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS fla_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(fla_msgs
  "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/Latency.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fla_msgs
)
_generate_msg_py(fla_msgs
  "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/ImageDetections.msg"
  "${MSG_I_FLAGS}"
  "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/Detection.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fla_msgs
)
_generate_msg_py(fla_msgs
  "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/FlightEvent.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fla_msgs
)
_generate_msg_py(fla_msgs
  "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/NodeStatus.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fla_msgs
)
_generate_msg_py(fla_msgs
  "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/ControlMessage.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fla_msgs
)
_generate_msg_py(fla_msgs
  "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/NodeList.msg"
  "${MSG_I_FLAGS}"
  "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/NodeStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fla_msgs
)
_generate_msg_py(fla_msgs
  "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/TelemString.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fla_msgs
)
_generate_msg_py(fla_msgs
  "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/FlightCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fla_msgs
)
_generate_msg_py(fla_msgs
  "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/Box.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fla_msgs
)
_generate_msg_py(fla_msgs
  "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/FlightStateTransition.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fla_msgs
)
_generate_msg_py(fla_msgs
  "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/WaypointList.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Float64.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fla_msgs
)
_generate_msg_py(fla_msgs
  "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/Detection.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fla_msgs
)
_generate_msg_py(fla_msgs
  "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/JoyDef.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fla_msgs
)
_generate_msg_py(fla_msgs
  "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/ProcessStatus.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fla_msgs
)

### Generating Services

### Generating Module File
_generate_module_py(fla_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fla_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(fla_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(fla_msgs_generate_messages fla_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/Latency.msg" NAME_WE)
add_dependencies(fla_msgs_generate_messages_py _fla_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/ImageDetections.msg" NAME_WE)
add_dependencies(fla_msgs_generate_messages_py _fla_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/FlightEvent.msg" NAME_WE)
add_dependencies(fla_msgs_generate_messages_py _fla_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/NodeStatus.msg" NAME_WE)
add_dependencies(fla_msgs_generate_messages_py _fla_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/ControlMessage.msg" NAME_WE)
add_dependencies(fla_msgs_generate_messages_py _fla_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/NodeList.msg" NAME_WE)
add_dependencies(fla_msgs_generate_messages_py _fla_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/ProcessStatus.msg" NAME_WE)
add_dependencies(fla_msgs_generate_messages_py _fla_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/TelemString.msg" NAME_WE)
add_dependencies(fla_msgs_generate_messages_py _fla_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/FlightCommand.msg" NAME_WE)
add_dependencies(fla_msgs_generate_messages_py _fla_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/Detection.msg" NAME_WE)
add_dependencies(fla_msgs_generate_messages_py _fla_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/FlightStateTransition.msg" NAME_WE)
add_dependencies(fla_msgs_generate_messages_py _fla_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/WaypointList.msg" NAME_WE)
add_dependencies(fla_msgs_generate_messages_py _fla_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/JoyDef.msg" NAME_WE)
add_dependencies(fla_msgs_generate_messages_py _fla_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sangita/project_ws/AER1516/src/acl-mapping/fla_msgs/msg/Box.msg" NAME_WE)
add_dependencies(fla_msgs_generate_messages_py _fla_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(fla_msgs_genpy)
add_dependencies(fla_msgs_genpy fla_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS fla_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fla_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fla_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(fla_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET nav_msgs_generate_messages_cpp)
  add_dependencies(fla_msgs_generate_messages_cpp nav_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(fla_msgs_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(fla_msgs_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fla_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fla_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(fla_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET nav_msgs_generate_messages_eus)
  add_dependencies(fla_msgs_generate_messages_eus nav_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(fla_msgs_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(fla_msgs_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fla_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fla_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(fla_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET nav_msgs_generate_messages_lisp)
  add_dependencies(fla_msgs_generate_messages_lisp nav_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(fla_msgs_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(fla_msgs_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fla_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fla_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(fla_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET nav_msgs_generate_messages_nodejs)
  add_dependencies(fla_msgs_generate_messages_nodejs nav_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(fla_msgs_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(fla_msgs_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fla_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fla_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fla_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(fla_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET nav_msgs_generate_messages_py)
  add_dependencies(fla_msgs_generate_messages_py nav_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(fla_msgs_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(fla_msgs_generate_messages_py geometry_msgs_generate_messages_py)
endif()
