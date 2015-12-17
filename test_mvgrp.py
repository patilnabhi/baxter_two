#!/usr/bin/env python

import rospy
from moveit_python import *
from geometry_msgs.msg import PoseStamped
# import baxter_interface

def test_mvgrp():
    rospy.init_node("moveit")
    g = MoveGroupInterface("left_arm", "base")

    pose_target = PoseStamped()
    pose_target.header.frame_id = "base"
    pose_target.header.stamp = rospy.Time.now()
    pose_target.pose.position.x = 0.3
    pose_target.pose.position.y = 0.0
    pose_target.pose.position.z = -0.3
    pose_target.pose.orientation.x = 1.0

    g.moveToPose(pose_target, "left_gripper", plan_only=False)
    print pose_target

if __name__=='__main__':
    try:
        test_mvgrp()
    except rospy.ROSInterruptException:
        pass
