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
```

<h2>rbsherpa_gazebo</h2>

Launch files and world files to start the models in gazebo

<h2>rbsherpa_sim_bringup</h2>

Launch files that execute the complete simulation of the robot


<h2>Simulating RB-Sherpa</h2>

1) Install the following dependencies:
  - [rbsherpa_common](https://github.com/RobotnikAutomation/rbsherpa_common)
  - [robotnik_sensors](https://github.com/RobotnikAutomation/robotnik_sensors)
  - [hector_gazebo](https://github.com/tu-darmstadt-ros-pkg/hector_gazebo)

2) Launch RB-Sherpa simulation:
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

3) Enjoy! You can use the topic "/robot/robotnik_base_control/cmd_vel" to control the RB-Sherpa robot.
