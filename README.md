# rbvogui_sim

Packages for the simulation of the RB-Vogui

<p align="center">
  <img src="https://github.com/RobotnikAutomation/rbvogui_sim/blob/melodic-master/doc/rbvogui_base.png" height="275" />
  <img src="https://github.com/RobotnikAutomation/rbvogui_sim/blob/melodic-master/doc/rbvogui_one_arm.png" height="275" />
  <img src="https://github.com/RobotnikAutomation/rbvogui_sim/blob/melodic-master/doc/rbvogui_xl_base.png" height="275" />
  <img src="https://github.com/RobotnikAutomation/rbvogui_sim/blob/melodic-master/doc/rbvogui_xl_gazebo.png" height="275" />

</p>

## Packages

This packages contains: 

### rbvogui_gazebo

Launch files and world files to start the models in gazebo

### rbvogui_sim_bringup

Launch files that execute the complete simulation of the robot

## Simulating RB-Vogui

### 1) Install the following dependencies:

This simulation has been tested using Gazebo 9 version. To facilitate the installation you can use the vcstool:

```bash
sudo apt-get install -y python3-vcstool
```

### 2) Create a workspace and clone the repository:

```bash
mkdir catkin_ws
cd catkin_ws
vcs import --input https://raw.githubusercontent.com/RobotnikAutomation/rbvogui_sim/melodic-master/doc/rbvogui_sim.repos
rosdep install --from-paths src --ignore-src -y
```

### 3) Install the controllers, robotnik_msgs and the rcomponent:


```bash
sudo dpkg -i src/rbvogui_common/libraries/*
```

### 4) Compile:

```bash
catkin build
source devel/setup.bash
```

Note: The package catkin-tools is need to compile with catkin build:
```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install python-catkin-tools
```

### 5) Run RB-Vogui simulation:


#### RB-Vogui

Set your robot kinematics to omni/ackermann (In case of ackermann, you will need twist2ackermann node enabled)
  
```bash
roslaunch rbvogui_sim_bringup rbvogui_complete.launch kinematics:=omni twist2ackermann:=false
```
 <p align="center">
  <img src="https://github.com/RobotnikAutomation/rbvogui_sim/blob/melodic-master/doc/rbvogui_base.png" height="275" />
</p>

#### RB-Vogui with one UR arm

In case you want to launch the rbvogui with an UR arm you can type the following command:
```bash
roslaunch rbvogui_sim_bringup rbvogui_complete.launch robot_xacro:=rbvogui_std_ur10.urdf.xacro launch_arm_control:=true arm_controllers:=arm_controller
```

 <p align="center">
  <img src="https://github.com/RobotnikAutomation/rbvogui_sim/blob/melodic-master/doc/rbvogui_one_arm.png" height="275" />
</p>

You can play with the arm by using the rqt_joint_trajectory:
```bash
ROS_NAMESPACE=robot rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller
```

Or even use moveit to plan trajectories:
```bash
ROS_NAMESPACE=robot roslaunch rbvogui_moveit_ur10 demo.launch
```

#### RB-Vogui XL model

If you prefer to launch the rbvogui XL, you can type:
```bash
roslaunch rbvogui_sim_bringup rbvogui_complete.launch robot_xacro:=rbvogui_xl.urdf.xacro
```

<p align="center">
  <img src="https://github.com/RobotnikAutomation/rbvogui_sim/blob/melodic-master/doc/rbvogui_xl_base.png" height="275" />
</p>

#### RB-Vogui XL with two UR arm

The rbvogui Xl can be launched with two UR arms, only this bi-arm (UR-10e) option is available:
```bash
roslaunch rbvogui_sim_bringup rbvogui_complete.launch launch_arm_control:=true robot_xacro:=rbvogui_xl.urdf.xacro
``` 

<p align="center">
  <img src="https://github.com/RobotnikAutomation/rbvogui_sim/blob/melodic-master/doc/rbvogui_xl_gazebo.png" height="275" />
</p>

To plan trajectories with the bi-arm robot you can type:

```bash
ROS_NAMESPACE=robot roslaunch rbvogui_xl_2ur10_e_moveit rbvogui_xl_moveit_config.launch
```

To switch between arms on RViz look for MotionPlanning > Planning Request > Planning Group and it will show you all the available groups (left_arm and right_arm).

### 6) Enjoy! 
You can use the topic "/robot/robotnik_base_control/cmd_vel" to control the RB-Vogui robot.
