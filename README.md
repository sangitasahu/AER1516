# AER1516 Winter 2022 Project

Link to draw.io file:   https://app.diagrams.net/#G1JaMom59_h-Wq3BAk28IzTx_z-1hYDaKI

## Set up the workspace
(We may modify the way we maintain alll the packages using rosinstall later, rosinstall essentially allows you to have packages developed seperately in different git repos (say, Matt develops a package in his github repo, you develop in another, rosinstall merges all the repos to create a ros workspace cloning the right version of the repos that now become dependencies for the project.-- The example below is how it works, the gazebo_drone.rosinstall essentially allows you to clone the mit repo for the entire gazebo and drone package)

This has been tested with Ubuntu 18.04/ROS Melodic

Install the following dependencies:
```
sudo apt-get install ros-"${ROS_DISTRO}"-gazebo-ros-pkgs ros-"${ROS_DISTRO}"-mavros-msgs ros-"${ROS_DISTRO}"-tf2-sensor-msgs ros-"${ROS_DISTRO}"-hector-sensors-description
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
wstool init src ~/project_ws/AER1516/gazebo_drone.rosinstall
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

# Git Project Structure

For AER1516 project, we will have the following Git branches:

1) main
2) testing (under main)
3) develop (under main)

The "develop" branch will be used during development and it contains the following 4 branches for four of us:
1) mapper
2) global_planner
3) convex_decomposition
4) local_planner

From now onwards always commit to the "develop" branch. Once implementation is over, we can merge our code to the "testing" branch for quadrotor simulation. Only at the end, the final code will be merged to the "main" branch.

If you want to develop small modules inside your package, you can create multiple branches under your develop/xxxxx/ branch.

Note: Local refers to your PC and remote refers to GitHub

## Setting up your Git branch

### Clone the Repo
```
$ git clone https://github.com/sangitasahu/AER1516.git
```
### Check the current branches 
```
$ cd AER1516/
$ git branch
$ git status
```
### Create a local branch (e.g. global_planner) if it doesn't exist
```
$ git checkout -b global_planner
$ git branch
```
Note: I have created branches named "mapper", "local_planner" and "convex_decomposition" in Github. You can use them.

### Add and commit all the current changes to the current local branch
```
$ git add .
$ git commit -m "my first commit!"
```
Give appropriate mesages for future tracking. We can also combine both the add and commit commands.
```
$ git commit -a -m "updated git commands"
```
### To connect the local branch to the remote branch
```
$ git branch --set-upstream-to=origin/global_planner global_planner
```
Note : Make sure there exists a branch with the local branch name in GitHub, else manually create it.

### Push the changes from local branch to remote branch
Note: Pulling is optional, pull from remote to local branch if required, else directly give push command for pushing changes from local to remote.
```
$ git pull
$ git push
```
Note: After every push, create a Pull request to the "develop" branch in GitHub by clicking "Create Pull Request" in your branch. Add a reviewer if possible, and then u can merge it to the "develop" branch.

Caution: Donot create or merge others' Pull requests. 

## Other Useful Commands
To Make or initialise a folder as Git Folder
```
$ git init
```
Pointing the local git folder to remote repository.
```
$ git remote add origin <http_address>
```
Update local repository with fresh data without merging them with current branch
```
$ git fetch 
```
List Branches
```
$ git branch  
```
For switching to any branch
```
$ git checkout develop
```
For deleting a branch
```
$ git branch -d <branch_name>
```
To check the changes
```
$ git diff 
```

## MOSEK Installation Steps
The MOSEK optimizer needs to be installed both systemwide as well as installing a Python package to interface with it. Installation steps are described in the documentation here: https://docs.mosek.com/9.3/install/installation.html for the general setup and here: https://docs.mosek.com/latest/pythonapi/install-interface.html for the Python interface
1. Request an academic license here with your school email: https://www.mosek.com/products/academic-licenses/. It's automated and you should get an email relatively quickly
2. Download and run the overall installer for your system from here: https://docs.mosek.com/9.3/install/installation.html#general-setup
3. Add a path variable for the runnable location. I did it by adding the following to my .bashrc. I have it installed in my home directory so if you put it elsewhere you'll have to update that obviously
```
# MOSEK path
export PATH=$PATH:~/mosek/mosek/9.3/tools/platform/linux64x86/bin
```
4. Dump the license file they send you in the root install folder below. Check it's set up properly by running msktestlic in a terminal
```
$HOME/mosek/mosek.lic
```
5. Install the Python interface via Pip in the Python environment your ROS setup is using
```
pip install Mosek
```
6. That's it! If you want you can test it by downloading and running one of the tutorial problems. There's a simple QP example here: https://docs.mosek.com/latest/pythonapi/tutorial-qo-shared.html . The general documentation for the Python interface is here if you're curious: https://docs.mosek.com/latest/pythonapi/intro_info.html
