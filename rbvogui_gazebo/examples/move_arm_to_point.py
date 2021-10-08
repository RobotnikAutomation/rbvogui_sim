#!/usr/bin/env python

import sys
import rospy
from math import pi
import moveit_commander
import geometry_msgs.msg

if __name__ == '__main__':

  rospy.init_node('move_arm_to_point_node')
  
  moveit_commander.roscpp_initialize(sys.argv)

  robot = moveit_commander.RobotCommander()
  scene = moveit_commander.PlanningSceneInterface()
  # rbvogui_xl_lift_ur10 -> ur10e_arm   rbvogui_xl_bi_arm -> left_arm / right_arm
  group = moveit_commander.MoveGroupCommander("arm")
  group.set_planner_id("RRTstar")
  group.set_planning_time(5)
  group.set_max_velocity_scaling_factor(0.5)

  planning_frame = group.get_planning_frame()
  print(planning_frame)

  print(robot.get_current_state())

  pose_goal = geometry_msgs.msg.Pose()
  pose_goal.orientation.w = 1.0
  pose_goal.position.x = 0.7
  pose_goal.position.y = 0.4
  pose_goal.position.z = 1.5
  group.set_pose_target(pose_goal)

  print("Sending goal...")
  group.go(wait=True)
  print("Goal successfull!")

  group.stop()
  group.clear_pose_targets()


