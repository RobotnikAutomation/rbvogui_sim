<h1>rbsherpa_sim</h1>

Packages for the simulation of the RB-Sherpa

<p align="center">
  <img src="https://github.com/RobotnikAutomation/rbsherpa_sim/blob/melodic-devel/rbsherpa_base.png" height="275" />
  <img src="https://github.com/RobotnikAutomation/rbsherpa_sim/blob/melodic-devel/rbsherpa_one_arm.png" height="275" />
    <img src="https://github.com/RobotnikAutomation/rbsherpa_sim/blob/melodic-devel/rbsherpa_xl_base.png" height="275" />
    <img src="https://github.com/RobotnikAutomation/rbsherpa_sim/blob/melodic-devel/rbsherpa_xl_gazebo.png" height="275" />

</p>

This packages contains: 

<h1>rbsherpa_gazebo</h1>

Launch files and world files to start the models in gazebo

<h1>rbsherpa_sim_bringup</h1>

Launch files that execute the complete simulation of the robot

<h1>Simulating RB-Sherpa</h1>

<h2> 1) Install the following dependencies: </h2>

This simulation has been tested using Gazebo 9 version. To facilitate the installation you can use the vcstool:

```bash
sudo apt-get install -y python3-vcstool
```

<h2> 2) Create a workspace and clone the repository: </h2>

```bash
mkdir catkin_ws
cd catkin_ws
vcs import --input https://raw.githubusercontent.com/RobotnikAutomation/rbsherpa_sim/melodic-devel/rbsherpa_sim.repos
rosdep install --from-paths src --ignore-src -y
```

<h2> 3) Install the controllers, robotnik_msgs and the rcomponent:  </h2>


```bash
sudo dpkg -i src/rbsherpa_common/libraries/ros-melodic-robotnik-msgs_1.1.0-0bionic_amd64.deb
sudo dpkg -i src/rbsherpa_common/libraries/ros-melodic-ackermann-drive-controller_0.0.0-0bionic_amd64.deb
sudo dpkg -i src/rbsherpa_common/libraries/ros-melodic-omni-drive-controller_0.0.0-0bionic_amd64.deb
sudo dpkg -i src/rbsherpa_common/libraries/ros-melodic-rcomponent_1.1.0-0bionic_amd64.deb
sudo dpkg -i src/rbsherpa_common/libraries/ros-melodic-robotnik-twist2ackermann_0.0.0-0bionic_amd64.deb
```

<h2> 4) Compile: </h2>

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

<h2> 5) Run RB-Sherpa simulation: </h2>


<h3>RB-Sherpa</h3>

  Set your robot kinematics to omni/ackermann (In case of ackermann, you will need twist2ackermann node enabled)
  
  ```bash
  roslaunch rbsherpa_sim_bringup rbsherpa_complete.launch kinematics:=omni twist2ackermann:=false
  ```
 <p align="center">
  <img src="https://github.com/RobotnikAutomation/rbsherpa_sim/blob/melodic-devel/rbsherpa_base.png" height="275" />
</p>

<h3>RB-Sherpa with one UR arm</h3>

  In case you want to launch the rbsherpa with an UR arm you can type the following command:
  ```bash
  roslaunch rbsherpa_sim_bringup rbsherpa_complete.launch robot_xacro:=rbsherpa_std_ur10.urdf.xacro launch_arm_control:=true arm_controllers:=arm_controller
  ```

 <p align="center">
  <img src="https://github.com/RobotnikAutomation/rbsherpa_sim/blob/melodic-devel/rbsherpa_one_arm.png" height="275" />
</p>

  You can play with the arm by using the rqt_joint_trajectory:
  ```bash
  ROS_NAMESPACE=robot rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller
  ```

  Or even use moveit to plan trajectories:
  ```bash
  ROS_NAMESPACE=robot roslaunch rbsherpa_moveit_ur10 demo.launch
  ```

<h3>RB-Sherpa XL model</h3>

  If you prefer to launch the rbsherpa XL, you can type:
  ```bash
  roslaunch rbsherpa_sim_bringup rbsherpa_complete.launch robot_xacro:=rbsherpa_xl.urdf.xacro
  ```

<p align="center">
  <img src="https://github.com/RobotnikAutomation/rbsherpa_sim/blob/melodic-devel/rbsherpa_xl_base.png" height="275" />
</p>

<h3>RB-Sherpa XL with two UR arm</h3>

  The rbsherpa Xl can be launched with two UR arms, only this bi-arm (UR-10e) option is available:
  ```bash
  roslaunch rbsherpa_sim_bringup rbsherpa_complete.launch launch_arm_control:=true robot_xacro:=rbsherpa_xl.urdf.xacro
  ``` 

<p align="center">
  <img src="https://github.com/RobotnikAutomation/rbsherpa_sim/blob/melodic-devel/rbsherpa_xl_gazebo.png" height="275" />
</p>

  To plan trajectories with the bi-arm robot you can type:

  ```bash
  ROS_NAMESPACE=robot roslaunch rbsherpa_xl_2ur10_e_moveit rbsherpa_xl_moveit_config.launch
  ```

To switch between arms on RViz look for MotionPlanning > Planning Request > Planning Group and it will show you all the available groups (left_arm and right_arm).

6) Enjoy! You can use the topic "/robot/robotnik_base_control/cmd_vel" to control the RB-Sherpa robot.
