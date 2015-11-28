#!/usr/bin/env python

import rospy
from moveit_python import *
from geometry_msgs.msg import PoseStamped
import baxter_interface

def picknplace():
    rospy.init_node("pnp")
    leftgripper = baxter_interface.Gripper('right')

    g = MoveGroupInterface("right_arm", "base")

    camview = PoseStamped()

    camview.header.frame_id = "base"
    camview.header.stamp = rospy.Time.now()
    camview.pose.position.x = 0.67
    camview.pose.position.y = -0.2
    camview.pose.position.z = 0.1
    camview.pose.orientation.x = 1.0

    g.moveToPose(camview, "right_gripper", plan_only=False)
    
    leftgripper.calibrate()
    camview.pose.position.z = -0.05
    g.moveToPose(camview, "right_gripper", plan_only=False)
    leftgripper.close()

    camview.pose.position.z = 0.3
    g.moveToPose(camview, "right_gripper", plan_only=False)

    camview.pose.position.x = 0.7
    camview.pose.position.y = 0.2
    g.moveToPose(camview, "right_gripper", plan_only=False)
    leftgripper.open()

if __name__=='__main__':
    try:
        picknplace()
    except rospy.ROSInterruptException:
        pass