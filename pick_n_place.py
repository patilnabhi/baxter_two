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

    left = baxter_interface.Gripper('left')


    z_drop = 0.07
    z_rise = 0.2
    camera_offset_x = -0.3
    camera_offset_y = -0.3
    cam_x = 0.6
    cam_y = -0.2
    cam_z = 0.6
    goal_x = 0.7
    goal_y = 0.6
    goal_z = 0.23
    obj_x = 0.654
    obj_y = 0.119
    obj_z = 0.01
    #This is the hard coded position of the right gripper camera for optimal view of the table
    camera_view = PoseStamped()
    camera_view.header.frame_id = "base"
    camera_view.header.stamp = rospy.Time.now()
    camera_view.pose.position.x = cam_x
    camera_view.pose.position.y = cam_y
    camera_view.pose.position.z = cam_z
    camera_view.pose.orientation.x = 1.0


    #does the camera read the z value of the object, or is it known based on the actual table height?
    #do we need to drop and rise z individually, or can we keep z constant?
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = "base"
    goal_pose.header.stamp = rospy.Time.now()
    goal_pose.pose.position.x = goal_x
    goal_pose.pose.position.y = goal_y
    goal_pose.pose.position.z = goal_z
    goal_pose.pose.orientation.x = 1.0

    test_object = PoseStamped()
    test_object.header.frame_id = "base"
    test_object.header.stamp = rospy.Time.now()
    test_object.pose.position.x = obj_x
    test_object.pose.position.y = obj_y
    test_object.pose.position.z = obj_z
    test_object.pose.orientation.x = 1.0

    left.calibrate()

    #This next block iterates through an array of PoseStampeds representing each object, 
    #and pick_n_places each

    ##WILL ADD ONCE SINGLE OBJECT IS TESTED AND WORKING ON MY SIM

#Pseudocode:

# def distance(x_obj, x_gripper, y_obj, y_gripper):
#     d = math.sqrt( (x_obj-x_gripper)^2 + (y_obj-y_gripper)^2 )  
#     return d

# while Array is not empty

    # Array = [obj1, obj2, obj3, obj4, obj5, obj6] of type PoseStamped
    # min_distance = 9999999 #some kind of large number

    # for i in range 1 to size(Array)
    #     d = distance(obj1.pose.position.x, goal_pose.pose.position.x, obj1.pose.position.y, goal_pose.pose.position.x)
    #     if d < min_distance:
    #         min_distance = d
    #         closest_obj = Array(i)

    # left_gripper.open()
    # group.moveToPose(camera_view, "right_gripper", plan_only=False)
    # group.moveToPose(closest_obj, "left_gripper", max_velocity_scaling_factor = 0.1, plan_only=False)
    # left_gripper.close()
    # group.moveToPose(goal_pose, "left_gripper", max_velocity_scaling_factor = 0.1, plan_only=False)
    # left_gripper.open()

    # scene.removeBox(closest_obj)
    # pop(Array)


    left.open()
    #Move camera to viewing pose
    group.moveToPose(camera_view, "right_gripper", plan_only=False)

    #Get object info
    
    #Offset camera to allow more room for left arm action
    cam_x = cam_x + camera_offset_x
    cam_y = cam_y + camera_offset_y
    camera_view.pose.position.x = cam_x
    camera_view.pose.position.y = cam_y
    group.moveToPose(camera_view, "right_gripper", plan_only=False)
    #Move left arm to object
    group.moveToPose(test_object, "left_gripper", plan_only=False)
    obj_z = obj_z - z_drop
    test_object.pose.position.z = obj_z
    group.moveToPose(test_object, "left_gripper", plan_only=False)
    #Close gripper
    left.close()
    #Move left arm to goal pose
    obj_z = obj_z + z_rise
    test_object.pose.position.z = obj_z
    group.moveToPose(test_object, "left_gripper", plan_only=False)
    group.moveToPose(goal_pose, "left_gripper", plan_only=False)
    #Drop object
    left.open()
    #Move camera back to viewing position
    cam_x = cam_x - camera_offset_x
    cam_y = cam_y - camera_offset_y
    camera_view.pose.position.x = cam_x
    camera_view.pose.position.y = cam_y
    group.moveToPose(camera_view, "right_gripper", plan_only=False)
    

if __name__=='__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass