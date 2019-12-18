rbsherpa_sim
=============

Packages for the simulation of the RB-Sherpa

Gazebo 9 required

<h2>rbsherpa_gazebo</h2>

Launch files and world files to start the models in gazebo

<h2>rbsherpa_sim_bringup</h2>

Launch files that execute the complete simulation of the robot


<h2>Simulating RB-Sherpa</h2>

1) Install the following dependencies:
  - [rbsherpa_common](https://github.com/RobotnikAutomation/rbsherpa_common)
  - [robotnik_sensors](https://github.com/RobotnikAutomation/robotnik_sensors)

2) Launch RB-Sherpa simulation with: <br>
  Set your robot model omni/ackermann
  - export ROBOT_MODEL=omni
  - roslaunch rbsherpa_sim_bringup rbsherpa_complete.launch

3) Enjoy! You can use the topic "/rbsherpa_a/rbsherpa_control/cmd_vel" to control the RB-Sherpa robot.
