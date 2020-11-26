rbsherpa_sim
=============

Packages for the simulation of the RB-Sherpa

Please note that we are using Gazebo 9 for this simulation and default version in kinetic is Gazebo 7. You will need to install Gazebo 9 to make simulation work.

Before installing Gazebo 9 you should unistall Gazebo 7:

```bash
>$ sudo apt-get remove ros-kinetic-gazebo* 
>$ sudo apt-get remove libgazebo* 
>$ sudo apt-get remove gazebo*
```

[Here](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install) you will find how to install Gazebo 9. Moreover, you will need to install manually some gazebo9 dependencies:

```bash
>$ sudo apt-get install ros-kinetic-gazebo9-ros-pkgs ros-kinetic-gazebo9-ros-control ros-kinetic-gazebo9*
sudo apt-get install -y python-wstool python3-rosdep
```

<h2>rbsherpa_gazebo</h2>

Launch files and world files to start the models in gazebo

<h2>rbsherpa_sim_bringup</h2>

Launch files that execute the complete simulation of the robot


<h2>Simulating RB-Sherpa</h2>

1) Create a workspace and clone the repository:

```bash
mkdir catkin_ws
cd catkin_ws
wstool init src
wstool merge -t src https://raw.githubusercontent.com/RobotnikAutomation/rbsherpa_sim/kinetic-master/rbsherpa_sim.rosinstall
wstool update -t src
rosdep install --from-paths src --ignore-src -y
```
3) Install the controllers:

```bash
sudo dpkg -i src/rbsherpa_common/libraries/ros-kinetic-ackermann-drive-controller_0.0.0-0xenial_amd64.deb
sudo dpkg -i src/rbsherpa_common/libraries/ros-kinetic-omni-drive-controller_0.0.0-0xenial_amd64.deb
sudo dpkg -i src/rbsherpa_common/libraries/ros-kinetic-rcomponent_1.1.0-0xenial_amd64.deb
sudo dpkg -i src/rbsherpa_common/libraries/ros-kinetic-robotnik-msgs_1.0.0-0xenial_amd64.deb
sudo dpkg -i src/rbsherpa_common/libraries/ros-kinetic-robotnik-twist2ackermann_0.0.0-0xenial_amd64.deb
```

4) Compile:

```bash
catkin build
source devel/setup.bash
```

5) Launch RB-Sherpa simulation:
<br>
  Set your robot kinematics to omni/ackermann (In case of ackermann, you will need twist2ackermann node enabled)
  
  ```bash
  roslaunch rbsherpa_sim_bringup rbsherpa_complete.launch kinematics:=omni twist2ackermann:=false
  ```

  In case you want to launch the rbsherpa with an UR arm you can type the following command:
  ```bash
  roslaunch rbsherpa_sim_bringup rbsherpa_complete.launch robot_xacro:=rbsherpa_std_ur10.urdf.xacro launch_arm_control:=true arm_controllers:=arm_controller
  ```

  Or you can play with the arm by using the rqt_joint_trajectory:
  ```bash
  ROS_NAMESPACE=robot rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller
  ```

  Or even use moveit to plan trajectories:
  ```bash
  ROS_NAMESPACE=robot roslaunch rbsherpa_moveit_ur10 demo.launch
  ```

  If you prefer to launch the rbsherpa XL, you can type:
  ```bash
  roslaunch rbsherpa_sim_bringup rbsherpa_complete.launch robot_xacro:=rbsherpa_xl.urdf.xacro
  ```

  The rbsherpa Xl can be launched with two UR arms, only this bi-arm option is available actually:
  ```bash
  roslaunch rbsherpa_sim_bringup rbsherpa_complete.launch robot_xacro:=rbsherpa_xl.urdf.xacro launch_arm_control:=true
  ``` 

  To plan trajectories with the bi-arm robot you can type:

  ```bash
  ROS_NAMESPACE=robot roslaunch rbsherpa_xl_2ur10_moveit_config rbsherpa_xl_moveit_config.launch
  ```

To switch between arms on RViz look for MotionPlanning > Planning Request > Planning Group and it will show you all the available groups (left_arm and right_arm).

6) Enjoy! You can use the topic "/robot/robotnik_base_control/cmd_vel" to control the RB-Sherpa robot.
