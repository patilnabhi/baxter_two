#!/usr/bin/env python

import rospy
import roslib; roslib.load_manifest("moveit_python")
from moveit_python import PlanningSceneInterface, MoveGroupInterface
from geometry_msgs.msg import PoseStamped, PoseArray
import baxter_interface
from moveit_python.geometry import rotate_pose_msg_by_euler_angles
from math import pi, sqrt

def picknplace():
    p = PlanningSceneInterface("base")
    g = MoveGroupInterface("both_arms", "base")
    gr = MoveGroupInterface("right_arm", "base")
    gl = MoveGroupInterface("left_arm", "base")
    leftgripper = baxter_interface.Gripper('left')
    bax = baxter_interface.RobotEnable()
    jts_both = ['left_e0', 'left_e1', 'left_s0', 'left_s1', 'left_w0', 'left_w1', 'left_w2', 'right_e0', 'right_e1', 'right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2']
    jts_right = ['right_e0', 'right_e1', 'right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2']
    jts_left = ['left_e0', 'left_e1', 'left_s0', 'left_s1', 'left_w0', 'left_w1', 'left_w2']

    p.clear()

    # Goto start state    
    pos = [-1.441426162661994, 0.8389151064712133, 0.14240920034028015, -0.14501001475655606, -1.7630090377446503, -1.5706376573674472, 0.09225918246029519, 0.43251610243732586, 0.97134058643113, 0.8794599572168327, -1.3021472716773106, -0.12228190858027599, 1.935400923056818, -2.6711046107800698]
    g.moveToJointPosition(jts_both, pos)

    temp = rospy.wait_for_message("Dpos", PoseArray)
    locs = temp.poses 
    locs_x = []
    locs_y = []
    orien = []

    for i in range(len(locs)):
        locs_x.append(0.63 + locs[i].position.x)
        locs_y.append(-0.2 + locs[i].position.y)
        orien.append(locs[i].position.z*pi/180)

    j=0
    for j in range(len(locs_x)):
        p.clear()
                    
        xn = locs_x[j]
        yn = locs_y[j]
        zn = -0.06
        thn = orien[j]

        print xn, yn

        # Add OTHER objects into planning scene
        i=0
        objlist = ['obj01', 'obj02', 'obj03', 'obj04', 'obj05', 'obj06', 'obj07', 'obj08']
        for i in range(j+1,len(locs_x)):
            p.addCube(objlist[i], 0.05, locs_x[i], locs_y[i], -0.07)
       
        p.waitForSync()

        leftgripper.calibrate()
        leftgripper.open()

        # Move right arm away            
        pos_2 = [1.805870011556845, 0.9850288102385463, -0.5588137158669255, -0.8373051976339934, -0.9004315064597952, 2.0874455972095483, -0.3359333266768987]            
        gr.moveToJointPosition(jts_right, pos_2)

        # Move left arm to pick object and pick object
        pickgoal = PoseStamped() 
        pickgoal.header.frame_id = "base"
        pickgoal.header.stamp = rospy.Time.now()
        pickgoal.pose.position.x = xn
        pickgoal.pose.position.y = yn
        pickgoal.pose.position.z = zn+0.1
        pickgoal.pose.orientation.x = 1.0
        pickgoal.pose.orientation.y = 0.0
        pickgoal.pose.orientation.z = 0.0
        pickgoal.pose.orientation.w = 0.0
        g.moveToPose(pickgoal, "left_gripper", plan_only=False)

        pickgoal.pose = rotate_pose_msg_by_euler_angles(pickgoal.pose, 0.0, 0.0, thn)
        g.moveToPose(pickgoal, "left_gripper", plan_only=False)
        
        pickgoal.pose.position.z = zn
        g.moveToPose(pickgoal, "left_gripper", max_velocity_scaling_factor = 0.1, plan_only=False)
        leftgripper.close()

        pickgoal.pose.position.z = zn+0.1
        g.moveToPose(pickgoal, "left_gripper", max_velocity_scaling_factor = 0.1, plan_only=False)

        # Move left arm to place object and place object            
        pos_place = [-1.441426162661994, 0.8389151064712133, 0.14240920034028015, -0.14501001475655606, -1.7630090377446503, -1.5706376573674472, 0.09225918246029519]            
        gl.moveToJointPosition(jts_left, pos_place)

        leftgripper.open()

if __name__=='__main__':
    try:
        rospy.init_node('pnp', anonymous=True)
        picknplace()

    except rospy.ROSInterruptException:
        pass