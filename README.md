# AER1516 Winter 2022 Project

Link to draw.io file:   https://app.diagrams.net/#G1JaMom59_h-Wq3BAk28IzTx_z-1hYDaKI

########################################################

##Set up the workspace
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
mkdir project_ws && cd ws && mkdir src && cd src
git clone https://github.com/sangitasahu/AER1516.git
wstool init
wstool merge -t src gazebo_drone.rosinstall
```

##Compile the code:
```
wstool update -j8
cd ..
catkin config -DCMAKE_BUILD_TYPE=Release
catkin_make
```
make sure you add the following line to your .bashrc file
( open a terminal at $home dir, and type gedit .bashrc ; scroll to the last line and add this line at the end ; save and exit) 
```
source PATH_TO_YOUR_WS/devel/setup.bash
```

And finally open 3 terminals and execute these commands the three different terminals:
```
roscore
roslaunch acl_sim start_world.launch
roslaunch acl_sim perfect_tracker_and_sim.launch
```