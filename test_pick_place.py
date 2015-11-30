#!/usr/bin/env python

import rospy
from moveit_python import *
from geometry_msgs.msg import PoseStamped
import baxter_interface

def picknplace():
    rospy.init_node("pnp")

    p = PlanningSceneInterface("base")

    p.addBox("goal_box", 0.3, 0.3, 0.22, 0.7, 0.6, 0.0)
    p.waitForSync()

    leftgripper = baxter_interface.Gripper('left')

    # Need to add code to clear planning scene:

    # Need to add table as an 'attached' object to planning scene:

    g = MoveGroupInterface("both_arms", "base") # define 'g'

    # Need to add initial position for both arms:

    camview = PoseStamped() # camview represents position of right arm to get view of objects on table

    camview.header.frame_id = "base"
    camview.header.stamp = rospy.Time.now()
    camview.pose.position.x = 0.6
    camview.pose.position.y = -0.2
    camview.pose.position.z = 0.6
    camview.pose.orientation.x = 1.0

    g.moveToPose(camview, "right_gripper", plan_only=False)   

    ### PicknPlace loop starts here until all objects are cleared:
    ## [Need to code this] Get nearest object location from left_gripper: 
    # 1. [Hanlin's opencv] Get Location of object(s) w.r.t right_camera (list of poses in x,y,z coord)
    # 2. Get Location of object(s) w.r.t base )
    # 3. Add objects to planning scene
    # 4. Find nearest object to left_gripper (smallest of sqrt(x^2+y^2+z^2)) for all objects
    # ============
    # Return x,y and z for nearest object:
    # ============
    
    # Assume x,y,z is found for an object
    xn = 0.64+0.014
    yn = -0.21+0.329
    zn = -0.06
    
    leftgripper.calibrate()
    leftgripper.open()

    pickgoal = PoseStamped() # Define position for left_arm to move to 

    pickgoal.header.frame_id = "base"
    pickgoal.header.stamp = rospy.Time.now()
    pickgoal.pose.position.x = xn
    pickgoal.pose.position.y = yn
    pickgoal.pose.position.z = zn+0.07
    pickgoal.pose.orientation.x = 1.0

    # # Need to add pose for right arm JUST before left arm comes in to grab objects:
    camview.pose.position.x = 0.3
    camview.pose.position.y = -0.5
    g.moveToPose(camview, "right_gripper", plan_only=False)
    g.moveToPose(pickgoal, "left_gripper", plan_only=False)

    pickgoal.pose.position.z = zn
    g.moveToPose(pickgoal, "left_gripper", max_velocity_scaling_factor = 0.1, plan_only=False)
    leftgripper.close()

    pickgoal.pose.position.z = zn+0.2
    g.moveToPose(pickgoal, "left_gripper", max_velocity_scaling_factor = 0.1, plan_only=False)

    # # Need to add code to remove object that is picked from planning scene:

    xg = 0.7
    yg = 0.6
    zg = 0.15

    placegoal = PoseStamped()

    placegoal.header.frame_id = "base"
    placegoal.header.stamp = rospy.Time.now()
    placegoal.pose.position.x = xg
    placegoal.pose.position.y = yg
    placegoal.pose.position.z = zg+0.08
    placegoal.pose.orientation.x = 1.0
    g.moveToPose(placegoal, "left_gripper", plan_only=False)
    
    # # Need to 'call' pose for right arm to capture view again:
    camview.pose.position.x = 0.3
    camview.pose.position.y = -0.5
    g.moveToPose(camview, "right_gripper", plan_only=False)

    # placegoal.pose.position.z = zg
    # g.moveToPose(placegoal, "left_gripper", plan_only=False)
    leftgripper.open()

    # continue for other objects

if __name__=='__main__':
    try:
        picknplace()
    except rospy.ROSInterruptException:
        pass