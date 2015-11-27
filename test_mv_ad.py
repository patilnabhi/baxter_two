#!/usr/bin/env python

import rospy
from moveit_python import *
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import RobotState
# import baxter_interface

def test_mv_ad():
    rospy.init_node("moveit")

    g = MoveGroupInterface("both_arms", "base")
    # gleft = MoveGroupInterface("left_arm", "base")
    # gright = MoveGroupInterface("right_arm", "base")
    # p = PlanningSceneInterface("base")

    xleft = 0.5
    yleft = 0.2
    zleft = -0.2

    pose_target_left = PoseStamped()
    pose_target_left.header.frame_id = "base"
    pose_target_left.header.stamp = rospy.Time.now()
    pose_target_left.pose.position.x = xleft
    pose_target_left.pose.position.y = yleft
    pose_target_left.pose.position.z = zleft + 0.2
    pose_target_left.pose.orientation.x = 1.0

    pose_target_right = PoseStamped()
    pose_target_right.header.frame_id = "base"
    pose_target_right.header.stamp = rospy.Time.now()
    pose_target_right.pose.position.x = xleft
    pose_target_right.pose.position.y = -yleft
    pose_target_right.pose.position.z = zleft + 0.2
    pose_target_right.pose.orientation.x = 1.0

    g.moveToPose(pose_target_left, "left_gripper", plan_only=False)
    g.moveToPose(pose_target_right, "right_gripper", plan_only=False)
    # print pose_target

if __name__=='__main__':
    try:
        test_mv_ad()
    except rospy.ROSInterruptException:
        pass