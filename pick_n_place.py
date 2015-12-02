#!/usr/bin/env python

import rospy
from moveit_python import *
import baxter_interface
from geometry_msgs.msg import PoseStamped

def main():

    rospy.init_node("grab_object")
    
    scene = PlanningSceneInterface("base")
    group = MoveGroupInterface("both_arms","base")
    scene.addBox("goal_box",0.3, 0.3, 0.22, 0.7, 0.6, 0.0)
    scene.waitForSync()

    left_gripper = baxter_interface.Gripper('left')

    #This is the hard coded position of the right gripper camera for optimal view of the table
    camera_view = PoseStamped()
    camera_view.header.frame_id = "base"
    camera_view.header.stamp = rospy.Time.now()
    camera_view.pose.position.x = 0.3
    camera_view.pose.position.y = -0.5
    camera_view.pose.position.z = 0.6
    camera_view.pose.orientation.x = 1.0

    z_drop = 0.07
    #does the camera read the z value of the object, or is it known based on the actual table height?
    #do we need to drop and rise z individually, or can we keep z constant?
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = "base"
    goal_pose.header.stamp = rospy.Time.now()
    goal_pose.pose.position.x = 0.7
    goal_pose.pose.position.y = 0.6
    goal_pose.pose.position.z = 0.23
    goal_pose.pose.orientation.x = 1.0

    test_object = PoseStamped()
    test_object.header.frame_id = "base"
    test_object.header.stamp = rospy.Time.now()
    test_object.pose.position.x = 0.654
    test_object.pose.position.y = 0.119
    test_object.pose.position.z = 0.01

    left_gripper.calibrate()

    #This iterates through an array of PoseStampeds representing each object, 
    #and pick_n_places each

    ##WILL ADD ONCE SINGLE OBJECT IS TESTED AND WORKING ON MY SIM

    left_gripper.open()
    group.moveToPose(camera_view, "right_gripper", plan_only=False)
    group.moveToPose(test_object, "left_gripper", max_velocity_scaling_factor = 0.1, plan_only=False)
    left_gripper.close()
    group.moveToPose(goal_pose, "left_gripper", max_velocity_scaling_factor = 0.1, plan_only=False)
    left_gripper.open()
    

if __name__=='__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass