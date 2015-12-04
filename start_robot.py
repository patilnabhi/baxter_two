#!/usr/bin/env python

import rospy
import roslib; roslib.load_manifest("moveit_python")
from moveit_python import *
from geometry_msgs.msg import PoseStamped
import baxter_interface

def start_robot():
    # p = PlanningSceneInterface("base")
    # p.attachBox

    jts = ['left_e0', 'left_e1', 'left_s0', 'left_s1', 'left_w0', 'left_w1', 'left_w2', 'right_e0', 'right_e1', 'right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2']
    pos = [-1.441426162661994, 0.8389151064712133, 0.14240920034028015, -0.14501001475655606, -1.7630090377446503, -1.5706376573674472, 0.09225918246029519, 0.43251610243732586, 0.97134058643113, 0.8794599572168327, -1.3021472716773106, -0.12228190858027599, 1.935400923056818, -2.6711046107800698]
    g = MoveGroupInterface("both_arms", "base")
    g.moveToJointPosition(jts, pos)

    # jts_right = ['right_e0', 'right_e1', 'right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2']
    # pos_2 = [1.805870011556845, 0.9850288102385463, -0.5588137158669255, -0.8373051976339934, -0.9004315064597952, 2.0874455972095483, -0.3359333266768987]
    # gr = MoveGroupInterface("right_arm", "base")
    # gr.moveToJointPosition(jts_right, pos_2)

    # sr = PoseStamped() # camview represents position of right arm to get view of objects on table
    # sr.header.frame_id = "base"
    # sr.header.stamp = rospy.Time.now()
    # sr.pose.position.x = 0.6
    # sr.pose.position.y = -0.05
    # sr.pose.position.z = 0.6
    # sr.pose.orientation.x = 1.0
    # sr.pose.orientation.y = 0.0
    # sr.pose.orientation.z = 0.0
    # sr.pose.orientation.w = 0.0

    # sl = PoseStamped()
    # sl.header.frame_id = "base"
    # sl.header.stamp = rospy.Time.now()
    # sl.pose.position.x = 0.75
    # sl.pose.position.y = 0.6
    # sl.pose.position.z = 0.2
    # sl.pose.orientation.x = 1.0
    # sl.pose.orientation.y = 0.0
    # sl.pose.orientation.z = 0.0
    # sl.pose.orientation.w = 0.0    
    
    # g.moveToPose(sl, "left_gripper", plan_only=False)
    # g.moveToPose(sr, "right_gripper", plan_only=False)

    

if __name__=='__main__':
    try:
        rospy.init_node('start_robot', anonymous=True)
        start_robot()       
    except rospy.ROSInterruptException:
        pass