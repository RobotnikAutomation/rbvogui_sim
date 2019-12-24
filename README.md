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

2) Launch RB-Sherpa simulation with: <br>
  Set your robot kinematics omni/ackermann (In case of ackermann, you will need twist2ackermann node enabled)
  - roslaunch rbsherpa_sim_bringup rbsherpa_complete.launch kinematics:=omni twist2ackermann:=false

3) Enjoy! You can use the topic "/robot/robotnik_base_control/cmd_vel" to control the RB-Sherpa robot.
