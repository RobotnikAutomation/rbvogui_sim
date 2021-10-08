#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult

if __name__ == '__main__':

    rospy.init_node('move_base_test_client')
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    
    print("Waiting move base action...")
    client.wait_for_server()
    print("move base action ready!")

    print("Sending move base goal")
    point = MoveBaseGoal()
    point.target_pose.header.frame_id = "robot_map"
    point.target_pose.header.stamp = rospy.Time.now()
    point.target_pose.pose.position.x = 1.0
    point.target_pose.pose.position.y = 1.0
    point.target_pose.pose.position.z = 0.0
    point.target_pose.pose.orientation.w = 1.0

    client.send_goal(point)
    print("Goal sent, waiting for result...")

    result = client.wait_for_result()
    print("Goal successfull!")
