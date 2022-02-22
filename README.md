# AER1516 Winter 2022 Project

Link to draw.io file:   https://app.diagrams.net/#G1JaMom59_h-Wq3BAk28IzTx_z-1hYDaKI

## Set up the workspace
(We may modify the way we maintain alll the packages using rosinstall later, rosinstall essentially allows you to have packages developed seperately in different git repos (say, Matt develops a package in his github repo, you develop in another, rosinstall merges all the repos to create a ros workspace cloning the right version of the repos that now become dependencies for the project.-- The example below is how it works, the gazebo_drone.rosinstall essentially allows you to clone the mit repo for the entire gazebo and drone package)

This has been tested with Ubuntu 18.04/ROS Melodic

Install the following dependencies:
```
sudo apt-get install ros-"${ROS_DISTRO}"-gazebo-ros-pkgs ros-"${ROS_DISTRO}"-mavros-msgs ros-"${ROS_DISTRO}"-tf2-sensor-msgs
```
```
sudo apt-get install python3-catkin-tools
python -m pip install pyquaternion
sudo apt install git

```
Create a workspace, and clone this repo and its dependencies:
```
mkdir project_ws && cd project_ws && mkdir src
git clone https://github.com/sangitasahu/AER1516.git
```
NOTE: if you get a prompt to enter username and password, you need PATs, else skip to initializing the workspace

How to Create Personal Access Token on GitHub:

From your GitHub account, go to Settings => Developer Settings => Personal Access Token => Generate New Token (Give your password) => Fillup the form (tick only repo Full control of private repositories ) => click Generate token => Copy the generated Token, (something like ghp_sFhFsSHhTzMDreGRLjmks4Tzuzgthdvfsrta)

For Linux, you need to configure the local GIT client with a username and email address,
```
git config --global user.name "your_github_username"
git config --global user.email "your_github_email"
git config -l
```
Once GIT is configured, we can begin using it to access GitHub.
```
git clone https:https://github.com/sangitasahu/AER1516.git 
> Cloning into `AER1516`...
Username for 'https://github.com' : username
Password for 'https://github.com' : give your personal access token here (paste it , but it wont be displayed)
```
Now cache the given record in your computer to remembers the token:
```
git config --global credential.helper cache
```

## Initialize the workspace
```
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

