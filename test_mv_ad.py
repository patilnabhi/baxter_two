#!/usr/bin/env python

import rospy
from moveit_python import *
from geometry_msgs.msg import PoseStamped

def test_mv_ad():
    rospy.init_node("moveit")

    g = MoveGroupInterface("both_arms", "base")
    
    xleft = 0.7
    yleft = 0.2
    zleft = 0.4

    pose_target_left = PoseStamped()
    pose_target_left.header.frame_id = "base"
    pose_target_left.header.stamp = rospy.Time.now()
    pose_target_left.pose.position.x = 0.7
    pose_target_left.pose.position.y = -0.1
    pose_target_left.pose.position.z = 0.0
    pose_target_left.pose.orientation.x = 1.0

    pose_target_right = PoseStamped()
    pose_target_right.header.frame_id = "base"
    pose_target_right.header.stamp = rospy.Time.now()
    pose_target_right.pose.position.x = 0.6
    pose_target_right.pose.position.y = 0.0
    pose_target_right.pose.position.z = 0.6
    pose_target_right.pose.orientation.x = 1.0

    g.moveToPose(pose_target_left, "left_gripper", plan_only=False)
    g.moveToPose(pose_target_right, "right_gripper", plan_only=False)
    # print pose_target

if __name__=='__main__':
    try:
        test_mv_ad()
    except rospy.ROSInterruptException:
        pass