#!/usr/bin/env python

import sys
import rospy
import moveit_commander
from math import pi

if __name__ == '__main__':

  rospy.init_node('move_arm_joint_by_joint_node')
  
  moveit_commander.roscpp_initialize(sys.argv)

  robot = moveit_commander.RobotCommander()
  scene = moveit_commander.PlanningSceneInterface()
  group = moveit_commander.MoveGroupCommander("arm")

  planning_frame = group.get_planning_frame()
  print(planning_frame)

  print(robot.get_current_state())

  joint_goal = group.get_current_joint_values()
  joint_goal[0] = 0
  joint_goal[1] = -pi/4
  joint_goal[2] = 0
  joint_goal[3] = -pi/2
  joint_goal[4] = 0
  joint_goal[5] = pi/3

  print("Sending goal...")
  group.go(joint_goal, wait=True)
  print("Goal successfull!")

  group.stop()



