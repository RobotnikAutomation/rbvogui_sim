# rbvogui_sim

Packages for the simulation of the RB-Vogui

<p align="center">
  <img src="doc/rbvogui_base.png" height="275" />
  <img src="doc/rbvogui_one_arm.png" height="275" />
  <img src="doc/rbvogui_xl_base.png" height="275" />
  <img src="doc/rbvogui_xl_gazebo.png" height="275" />
  <img src="doc/rbvogui_xl_lift.png" height="275" />
</p>

## Packages

This packages contains: 

### rbvogui_gazebo

Launch files and world files to start the models in gazebo

### rbvogui_sim_bringup

Launch files that execute the complete simulation of the robot

## Simulating RB-Vogui

### 1) Install the following dependencies:

This simulation has been tested using Gazebo 9 version. To facilitate the installation you can use the ```vcstool```:

```bash
sudo apt-get install -y python3-vcstool
```

Install ```catkin_tools``` in order to compile the workspace

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install python-catkin-tools
```

Install ```rqt_joint_trajectory_controller``` to move the arm joint by joint

```bash
ROS_NAMESPACE=robot rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller
```

### 2) Create a workspace and clone the repository:

```bash
mkdir catkin_ws
cd catkin_ws
```

For the latest version:

```bash
mkdir catkin_ws
cd catkin_ws
vcs import --input https://raw.githubusercontent.com/RobotnikAutomation/rbvogui_sim/melodic-devel/repos/rbvogui_sim_devel.repos
rosdep install --from-paths src --ignore-src -y
```
<!--
For the stable version (some latest features may be not available):

```bash
mkdir catkin_ws
cd catkin_ws
vcs import --input https://raw.githubusercontent.com/RobotnikAutomation/rbvogui_sim/melodic-devel/repos/rbvogui_sim.repos
rosdep install --from-paths src --ignore-src -y
``` -->

### 3) Install the controllers, robotnik_msgs and the rcomponent:


```bash
cd ~/catkin_ws
sudo dpkg -i src/rbvogui_common/libraries/*
```

### 4) Compile:

```bash
cd ~/catkin_ws
catkin build
source devel/setup.bash
```

### 5) Run RB-Vogui simulation:

These are the different configurations available:

- Vogui
- Vogui with UR-10e arm
- Vogui XL
- Vogui XL with left and right UR10e arm
- Vogui XL with UR-10e arm and Ewellix lift

### 5.1 RB-Vogui

Set your robot kinematics to omni/ackermann (In case of ackermann, you will need twist2ackermann node enabled)
  
```bash
roslaunch rbvogui_sim_bringup rbvogui_complete.launch kinematics:=omni twist2ackermann:=false
```
<p align="center">
  <img src="doc/rbvogui_base.png" height="275" />
</p>

### 5.2 RB-Vogui with UR10e arm

In case you want to launch the rbvogui with an UR arm you can type the following command:
```bash
  roslaunch rbvogui_sim_bringup rbvogui_complete.launch robot_xacro:=rbvogui_std_ur10.urdf.xacro launch_arm:=true arm_manufacturer:=ur arm_model:=ur10
```

<p align="center">
  <img src="doc/rbvogui_one_arm.png" height="275" />
</p>

You can play with the arm by using the rqt_joint_trajectory:
```bash
ROS_NAMESPACE=robot rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller
```

Or even use moveit to plan trajectories:
```bash
ROS_NAMESPACE=robot roslaunch rbvogui_moveit_ur10 demo.launch
```

### 5.3 RB-Vogui XL

If you prefer to launch the rbvogui XL, you can type:
```bash
roslaunch rbvogui_sim_bringup rbvogui_complete.launch robot_xacro:=rbvogui_xl.urdf.xacro
```

<p align="center">
  <img src="doc/rbvogui_xl_base.png" height="275" />
</p>

### 5.4 RB-Vogui XL with two UR10e arm

The rbvogui Xl can be launched with two UR arms, only this bi-arm (UR-10e) option is available:
```bash
roslaunch rbvogui_sim_bringup rbvogui_complete.launch robot_xacro:=rbvogui_xl.urdf.xacro launch_arm:=true arm_manufacturer:=ur arm_model:=bi_ur10e
``` 

<p align="center">
  <img src="doc/rbvogui_xl_gazebo.png" height="275" />
</p>


You can play with the arms by using the rqt_joint_trajectory:
```bash
ROS_NAMESPACE=robot rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller
```

To plan trajectories with the bi-arm robot you can type:

```bash
ROS_NAMESPACE=robot roslaunch rbvogui_xl_2ur10_e_moveit rbvogui_xl_moveit_config.launch
```

To switch between arms on RViz look for MotionPlanning > Planning Request > Planning Group and it will show you all the available groups (left_arm and right_arm).

### 5.5 RB-Vogui XL with UR10e arm and Ewellix lift

The rbvogui Xl can also be launched with an UR-10e arm and an Ewellix lift:


```bash
roslaunch rbvogui_sim_bringup rbvogui_complete.launch robot_xacro:=rbvogui_xl_lift_ur10e.urdf.xacro launch_arm:=true arm_manufacturer:=ur arm_model:=lift_ur10e
``` 

<p align="center">
  <img src="doc/rbvogui_xl_lift.png" height="275" />
</p>

You can play with the arm by using the rqt_joint_trajectory:
```bash
ROS_NAMESPACE=robot rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller
```

To plan trajectories with the robot you can type:

```bash
ROS_NAMESPACE=robot roslaunch rbvogui_xl_lift_ur10e_moveit demo.launch
```

To control the lift, you can type:

```bash
rostopic pub /robot/lift_controller/command std_msgs/Float64 "data: 0.2"
```

<!-- 
### 6) Enjoy!

You can use the topic ```/robot/robotnik_base_control/cmd_vel ``` to control the RB-Vogui robot. -->

### 6) Localization and navigation

Launch localization and navigation:

```bash
roslaunch rbvogui_sim_bringup rbvogui_complete.launch run_localization:=true run_navigation:=true
```

Launch mapping and create a map:

```bash
roslaunch rbvogui_sim_bringup rbvogui_complete.launch run_localization:=true run_mapping:=true
```

Save the map

```bash
ROS_NAMESPACE=robot rosrun map_server map_saver -f demo
```

## Docker usage

In order to run this simulation you will need nvidia graphical accelation

### Installation of required files
- [docker](https://docs.docker.com/engine/install/ubuntu/)
- [nvidia-docker](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker)
- nvidia-drivers

### Usage

```bash
git clone https://github.com/RobotnikAutomation/rbvogui_sim.git
cd rbvogui_sim
git checkout melodic-devel
docker/simulation-in-container-run.sh

```

#### Selecting the robot model

You can select the robot, the launch file of package using the optional arguments on launch
By default the selected robot is `rbvogui`

```bash
docker/simulation-in-container-run.sh --help
```

```
ROBOTNIK AUTOMATION S.L.L. 2021

Simulation of RB VOGUI using docker

Usage:
docker/simulation-in-container-run.sh [OPTIONS]

Optional arguments:
 --robot -r ROBOT       Select robot to simulate
                        Valid robots:
                            rb_vogui_one_ur_arm rb_vogui_xl_two_ur_arms rb_vogui rb_vogui_xl
                        default: rb_vogui

 --launch -l            Select launch file
                        default: rbvogui_complete.launch kinematics:=omni twist2ackermann:=false

 --package -p           Select ros package
                        default: rbvogui_sim_bringup

 --ros-port -u PORT     Host ros port
                        default: 11345

 --gazebo-port -g PORT  Host ros port
                        default: 11345

 -h, --help             Shows this help

```

**RB Vogui with one UR arm**
```bash
docker/simulation-in-container-run.sh --robot rb_vogui_one_ur_arm
```
***IMPORTANT:*** This simulation starts paused, please remember to press play button on gazebo after few seconds

**RB Vogui XL**
```bash
docker/simulation-in-container-run.sh --robot rb_vogui_xl
```
***IMPORTANT:*** This simulation starts paused, please remember to press play button on gazebo after few seconds

**RB Vogui XL with UR arms**
```bash
docker/simulation-in-container-run.sh --robot rb_vogui_xl_two_ur_arms
```
***IMPORTANT:*** This simulation starts paused, please remember to press play button on gazebo after few seconds

#### Manual Build

If you wish to build manually the image without the use of the script use one the following commands:

**Optiona A**
```bash
cd docker
docker build -f Dockerfile ..
```
**Option B**
```bash
docker build -f docker/Dockerfile .
```

#### Notes

- This is docker requires a graphical interface
- The ros master uri is accesible outside the container, so in the host any ros command should work
- You could also run a roscore previous to launch the simulation in order to have some processes on the host running
- if you want to enter on the container use the following command in another terminal
```bash
docker container exec -it rb_vogui_sim_instance bash
```
- In order to exit you have to 2 options
1. Close `gazebo` and `rviz` and wait a bit
2. execute in another terminal:
```bash
docker container rm --force rb_vogui_sim_instance
```