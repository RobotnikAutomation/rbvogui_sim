rbsherpa_hl_sim
=============

Packages for the simulation of the RBSherpa HL

<h2>rbsherpa_hl_gazebo</h2>

Launch files and world files to start the models in gazebo

<h2>rbsherpa_hl_sim_bringup</h2>

Launch files that execute the complete simulation of the robot


<h2>Simulating RBSherpa HL</h2>

1) Install the following dependencies:
  - [rbsherpa_hl_common](https://github.com/RobotnikAutomation/rbsherpa_hl_common)
  - [robotnik_sensors](https://github.com/RobotnikAutomation/robotnik_sensors)

2) Launch RBSherpa HL simulation with: <br>
  Set your robot model omni/ackermann
  - export ROBOT_MODEL=omni
  - roslaunch rbsherpa_hl_sim_bringup rbsherpa_hl_complete.launch

3) Enjoy! You can use the topic "/rbsherpa_hl_a/rbsherpa_hl_control/cmd_vel" to control the RBSherpa HL robot.
