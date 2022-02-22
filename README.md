# AER1516 Winter 2022 Project

Link to draw.io file:   https://app.diagrams.net/#G1JaMom59_h-Wq3BAk28IzTx_z-1hYDaKI

## Set up the workspace

This has been tested with Ubuntu 18.04/ROS Melodic

Install the following dependencies:
```
sudo apt-get install ros-"${ROS_DISTRO}"-gazebo-ros-pkgs ros-"${ROS_DISTRO}"-mavros-msgs ros-"${ROS_DISTRO}"-tf2-sensor-msgs
```
```
python -m pip install pyquaternion
```
Create a workspace, and clone this repo and its dependencies:
```
mkdir project_ws && cd project_ws && mkdir src
git clone https://github.com/sangitasahu/AER1516.git
wstool init src
wstool merge -t src ~/project_ws/AER1516/gazebo_drone.rosinstall
```

## Compile the code:
```
cd src
wstool update -j8
cd ..
catkin config -DCMAKE_BUILD_TYPE=Release
catkin_make
```
## Source your workspace
Make sure you add the following line to your .bashrc file
( open a terminal at $home dir, and type gedit .bashrc ; scroll to the last line and add this line at the end ; save and exit) 
```
source PATH_TO_YOUR_WS/devel/setup.bash
```
(HINT: if you have sourced your workspace properly, typing roscd in a terminal must change the terminal directory to PATH_TO_YOUR_WS/devel)

## Launch the environment and Import the Drone into the Environment. 

Open 3 terminals and execute these commands the three different terminals:
```
roscore
roslaunch acl_sim start_world.launch
roslaunch acl_sim perfect_tracker_and_sim.launch
```
## Rostopics published
Open a new terminal again to check for all the topics that are being published: 
```
rostopic list
```
## Topics are:
```
/SQ01s/camera/cloud
/SQ01s/camera/depth/camera_info
/SQ01s/camera/depth/image_rect_raw
/SQ01s/camera/rgb/camera_info
/SQ01s/camera/rgb/image_raw
/SQ01s/camera/rgb/image_raw/compressed
/SQ01s/camera/rgb/image_raw/compressed/parameter_descriptions
/SQ01s/camera/rgb/image_raw/compressed/parameter_updates
/SQ01s/camera/rgb/image_raw/compressedDepth
/SQ01s/camera/rgb/image_raw/compressedDepth/parameter_descriptions
/SQ01s/camera/rgb/image_raw/compressedDepth/parameter_updates
/SQ01s/camera/rgb/image_raw/theora
/SQ01s/camera/rgb/image_raw/theora/parameter_descriptions
/SQ01s/camera/rgb/image_raw/theora/parameter_updates
/SQ01s/goal
/SQ01s/marker
/SQ01s/state
/clock
/gazebo/link_states
/gazebo/model_states
/gazebo/parameter_descriptions
/gazebo/parameter_updates
/gazebo/set_link_state
/gazebo/set_model_state
/rosout
/rosout_agg
/tf
```
NOTE: 
1) SQ01s is the name of the quadrotor in the simulation
2) Set goal by publishing to /SQ01/goal
3) Check status by subscribing to /SQ01/state
4) ~/camera/ gives camera info, all data already transformed

Visualize the camera data by : 
''

