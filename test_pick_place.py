#!/usr/bin/env python

import rospy
import roslib; roslib.load_manifest("moveit_python")
from moveit_python import *
from geometry_msgs.msg import PoseStamped, PoseArray
import baxter_interface
from moveit_python.geometry import *
from math import pi

def start_robot():

    p = PlanningSceneInterface("base")
    p.clear()

    g = MoveGroupInterface("both_arms", "base")

    sr = PoseStamped() # camview represents position of right arm to get view of objects on table
    sr.header.frame_id = "base"
    sr.header.stamp = rospy.Time.now()
    sr.pose.position.x = 0.6
    sr.pose.position.y = -0.2
    sr.pose.position.z = 0.6
    sr.pose.orientation.x = 1.0
    sr.pose.orientation.y = 0.0
    sr.pose.orientation.z = 0.0
    sr.pose.orientation.w = 0.0

    sl = PoseStamped()
    sl.header.frame_id = "base"
    sl.header.stamp = rospy.Time.now()
    sl.pose.position.x = 0.7
    sl.pose.position.y = 0.65
    sl.pose.position.z = 0.2
    sl.pose.orientation.x = 1.0
    sl.pose.orientation.y = 0.0
    sl.pose.orientation.z = 0.0
    sl.pose.orientation.w = 0.0    

    g.moveToPose(sr, "right_gripper", plan_only=False)
    g.moveToPose(sl, "left_gripper", plan_only=False)

def picknplace(data):
    # rospy.init_node("pnp")
    start_robot()

    p = PlanningSceneInterface("base")
    p.clear()
    
    leftgripper = baxter_interface.Gripper('left')

    g = MoveGroupInterface("both_arms", "base") # define 'g'

    # Need to add initial position for both arms:

    camview = PoseStamped() # camview represents position of right arm to get view of objects on table

    camview.header.frame_id = "base"
    camview.header.stamp = rospy.Time.now()
    camview.pose.position.x = 0.6
    camview.pose.position.y = -0.2
    camview.pose.position.z = 0.6
    camview.pose.orientation.x = 1.0
    camview.pose.orientation.y = 0.0
    camview.pose.orientation.z = 0.0
    camview.pose.orientation.w = 0.0

    g.moveToPose(camview, "right_gripper", plan_only=False)

    # camview2 = PoseStamped()
    # camview2.header.frame_id = "base"
    # camview2.header.stamp = rospy.Time.now()
    # camview2.pose = rotate_pose_msg_by_euler_angles(camview.pose, 0.0, 0.0, 0.75*pi) 
    # g.moveToPose(camview2, "right_gripper", plan_only=False)


    ## PicknPlace loop starts here until all objects are cleared:
    # [Need to code this] Get nearest object location from left_gripper: 
    # 1. [Hanlin's opencv] Get Location of object(s) w.r.t right_camera (list of poses in x,y,z coord)
    # 2. Get Location of object(s) w.r.t base )
    # 3. Add objects to planning scene
    # 4. Find nearest object to left_gripper (smallest of sqrt(x^2+y^2+z^2)) for all objects
    # ============
    # Return x,y and z for nearest object:
    # ============
    
    # Assume x,y,z is found for an object
    xn = 0.61+data.poses[0].position.x
    yn = -0.19+data.poses[0].position.y
    zn = -0.06

    leftgripper.calibrate()
    leftgripper.open()

    pickgoal = PoseStamped() # Define position for left_arm to move to 

    pickgoal.header.frame_id = "base"
    pickgoal.header.stamp = rospy.Time.now()
    pickgoal.pose.position.x = xn
    pickgoal.pose.position.y = yn
    pickgoal.pose.position.z = zn+0.1
    pickgoal.pose.orientation.x = 1.0  

    g.moveToPose(pickgoal, "left_gripper", plan_only=False)

    pickgoal.pose.position.z = zn
    g.moveToPose(pickgoal, "left_gripper", max_velocity_scaling_factor = 0.1, plan_only=False)
    leftgripper.close()

    pickgoal.pose.position.z = zn+0.09
    g.moveToPose(pickgoal, "left_gripper", max_velocity_scaling_factor = 0.1, plan_only=False)

    xg = 0.7
    yg = 0.65
    zg = 0.0

    placegoal = PoseStamped()

    placegoal.header.frame_id = "base"
    placegoal.header.stamp = rospy.Time.now()
    placegoal.pose.position.x = xg
    placegoal.pose.position.y = yg
    placegoal.pose.position.z = zg+0.1
    placegoal.pose.orientation.x = 1.0
    g.moveToPose(placegoal, "left_gripper", plan_only=False)
    
    placegoal.pose.position.z = zg-0.03
    g.moveToPose(placegoal, "left_gripper", plan_only=False)

    leftgripper.open()

    placegoal.pose.position.z = zg+0.2
    g.moveToPose(placegoal, "left_gripper", plan_only=False)
    

    # continue for other objects
    # p.removeAttachedObject("table")

def img_callback():
    rospy.init_node('img_callback', anonymous=True)
    rospy.Subscriber("Dpos", PoseArray, picknplace)
    rospy.spin()

if __name__=='__main__':
    try:
        # rospy.init_node('pnp', anonymous=True)
        # start_robot()
        img_callback()
        # picknplace(obj_loc_list)

    except rospy.ROSInterruptException:
        pass