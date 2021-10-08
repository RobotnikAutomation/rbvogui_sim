#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import geometry_msgs.msg

if __name__ == '__main__':

  rospy.init_node('get_arm_pose')
  
  moveit_commander.roscpp_initialize(sys.argv)

  robot = moveit_commander.RobotCommander()
  scene = moveit_commander.PlanningSceneInterface()
  group = moveit_commander.MoveGroupCommander("arm")

  pose = group.get_current_pose()

  joints = group.get_current_joint_values()

  print(" =========== End effector ==========")
  print(pose.pose)

  print("============== Joints ==============")
  print("joints")
  print("  j0: " + str(joints[0]))
  print("  j1: " + str(joints[1]))
  print("  j2: " + str(joints[2]))
  print("  j3: " + str(joints[3]))
  print("  j4: " + str(joints[4]))
  print("  j5: " + str(joints[5]))






