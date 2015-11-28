#!/usr/bin/env python

import rospy
from moveit_python import *
from moveit_msgs.msg import Grasp
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectoryPoint
import baxter_interface

def test_grasp():
    rospy.init_node("grasp")

    # gr = Grasp()

    # xleft = 0.5
    # yleft = 0.2
    # zleft = -0.2

    # grasp_pose = PoseStamped()
    # grasp_pose.header.frame_id="base"
    # grasp_pose.pose.position.x = xleft
    # grasp_pose.pose.position.y = yleft
    # grasp_pose.pose.position.z = zleft
    # grasp_pose.pose.orientation.x = 1.0

    # grasp = Grasp()
  
    # grasp.grasp_pose = grasp_pose
    # grasp.pre_grasp_approach.direction.vector.y = 0
    # grasp.pre_grasp_approach.direction.vector.x = 0
    # grasp.pre_grasp_approach.direction.vector.z = 1
    # grasp.pre_grasp_approach.direction.header.frame_id = "base"
    # grasp.pre_grasp_approach.min_distance = 0.01
    # grasp.pre_grasp_approach.desired_distance = 0.25

    # grasp.post_grasp_retreat.direction.header.frame_id = "base"
    # grasp.pre_grasp_approach.direction.vector.y = 0
    # grasp.pre_grasp_approach.direction.vector.x = 0
    # grasp.pre_grasp_approach.direction.vector.z = -1
    # grasp.post_grasp_retreat.min_distance = 0.01
    # grasp.post_grasp_retreat.desired_distance = 0.25

    # grasp.pre_grasp_posture.header.frame_id="base"
    # grasp.pre_grasp_posture.joint_names.append("left_gripper_base")
    # pre_point = JointTrajectoryPoint()
    # pre_point.positions.append(0.0095)
    # grasp.pre_grasp_posture.points.append(pre_point)

    # grasp.grasp_posture.header.frame_id="base"
    # grasp.grasp_posture.joint_names.append("left_gripper_base")
    # point = JointTrajectoryPoint()
    # point.positions.append(-0.0125)  
    # grasp.grasp_posture.points.append(point)

    leftgripper = baxter_interface.Gripper('right')
    leftgripper.calibrate()
    leftgripper.close()
    leftgripper.open()
    # leftgripper.close()


if __name__=='__main__':
    try:
        test_grasp()
    except rospy.ROSInterruptException:
        pass