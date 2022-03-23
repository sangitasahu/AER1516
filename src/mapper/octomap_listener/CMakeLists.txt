cmake_minimum_required(VERSION 3.0.2)
project(octomap_listener)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs octomap_ros octomap_server octomap_msgs sensor_msgs geometry_msgs snapstack_msgs tf)
find_package(octomap REQUIRED)

catkin_package()
include_directories( include ${catkin_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS})

add_executable(listener src/octomapListner.cpp)
target_link_libraries(listener ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES})
