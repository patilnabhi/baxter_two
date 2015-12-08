#!/usr/bin/env python

import rospy
import roslib; roslib.load_manifest("moveit_python")
from moveit_python import PlanningSceneInterface, MoveGroupInterface
from geometry_msgs.msg import PoseStamped, PoseArray
import baxter_interface
from moveit_python.geometry import rotate_pose_msg_by_euler_angles
from math import pi, sqrt

def del_meth(somelist, rem):
    for i in rem:
        somelist[i]='!' 
    for i in range(0,somelist.count('!')):
        somelist.remove('!')
    return somelist

def picknplace():
    p = PlanningSceneInterface("base")
    g = MoveGroupInterface("both_arms", "base")
    gr = MoveGroupInterface("right_arm", "base")
    gl = MoveGroupInterface("left_arm", "base")
    leftgripper = baxter_interface.Gripper('left')
    leftgripper.calibrate()
    leftgripper.open()
    jts_both = ['left_e0', 'left_e1', 'left_s0', 'left_s1', 'left_w0', 'left_w1', 'left_w2', 'right_e0', 'right_e1', 'right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2']
    jts_right = ['right_e0', 'right_e1', 'right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2']
    jts_left = ['left_e0', 'left_e1', 'left_s0', 'left_s1', 'left_w0', 'left_w1', 'left_w2']

    p.clear()
    p.attachBox('table', 0.7, 1.27, 0.54, 0.65, -0.2, -0.38, 'base', touch_links=['pedestal'])

    # # Goto start state 
    
    # gl.moveToPose(stleft, "left_gripper", plan_only=False) 
    # # stleft.pose.position.y = 0.4
    # # gl.moveToPose(stleft, "left_gripper", plan_only=False) 
    # # stleft.pose.position.y = 0.2
    # # gl.moveToPose(stleft, "left_gripper", plan_only=False) 
    # stleft.pose.position.y = 0.0
    # gl.moveToPose(stleft, "left_gripper", plan_only=False)
    # stleft.pose.position.z = -0.06
    # gl.moveToPose(stleft, "left_gripper", plan_only=False)

        
    # g.moveToJointPosition(jts_right,joint_pos)

    lpos1 = [-1.441426162661994, 0.8389151064712133, 0.14240920034028015, -0.14501001475655606, -1.7630090377446503, -1.5706376573674472, 0.09225918246029519]
    gl.moveToJointPosition(jts_left, lpos1, plan_only=False)
    
    lpos2 = [-0.949534106616211, 1.4994662184448244, -0.6036214393432617, -0.7869321432861328, -2.4735440176391603, -1.212228316241455, -0.8690001153442384]
    

    # stright = PoseStamped() 
    # stright.header.frame_id = "base"
    # stright.header.stamp = rospy.Time.now()
    # stright.pose.position.x = 0.55
    # stright.pose.position.y = 0.0
    # stright.pose.position.z = 0.3
    # stright.pose.orientation.x = 1.0
    # stright.pose.orientation.y = 0.0
    # stright.pose.orientation.z = 0.0
    # stright.pose.orientation.w = 0.0
    
    # stright.pose.position.y = -0.4
    # gr.moveToPose(stright, "right_gripper", plan_only=False)
    rpos1 = [1.7238109084167481, 1.7169079948791506, 0.36930587426147465, -0.33249033539428713, -1.2160632682067871, 1.668587600115967, -1.810097327636719]
    rpos2 = [1.8342575250183106, 1.8668546167236328, -0.45674277907104494, -0.21667478604125978, -1.2712865765075685, 1.7472041154052735, -2.4582042097778323]            
    gr.moveToJointPosition(jts_right, rpos1, plan_only=False)
    
    # gr.moveToPose(stright, "right_gripper", plan_only=False)
    temp = rospy.wait_for_message("Dpos", PoseArray)
    locs = temp.poses 
    locs_x = []
    locs_y = []
    orien = []

    for i in range(len(locs)):
        locs_x.append(0.565 + locs[i].position.x)
        locs_y.append(-0.011 + locs[i].position.y)
        orien.append(locs[i].position.z*pi/180)

    print locs_y
    ind_rmv = []
    for i in range(0,len(locs)):
        if (locs_y[i] > 0.47):
            ind_rmv.append(i)
            continue
        for j in range(i,len(locs)):
            if not (i == j):
                if sqrt((locs_x[i] - locs_x[j])**2 + (locs_y[i] - locs_y[j])**2)<0.01:
                    ind_rmv.append(i)
    
    locs_x = del_meth(locs_x, ind_rmv)
    locs_y = del_meth(locs_y, ind_rmv)
    orien = del_meth(orien, ind_rmv)    

    j=0
    for j in range(len(locs_x)):
        p.clear()
        p.attachBox('table', 0.7, 1.27, 0.54, 0.65, -0.2, -0.38, 'base', touch_links=['pedestal'])           
        xn = locs_x[j]
        yn = locs_y[j]
        zn = -0.06
        thn = orien[j]
        if thn > pi/4:
            thn = -1*(thn%(pi/4))

        # print xn, yn

        # Add OTHER objects into planning scene
        i=0
        objlist = ['obj01', 'obj02', 'obj03', 'obj04', 'obj05', 'obj06', 'obj07', 'obj08', 'obj09', 'obj10', 'obj11']
        for i in range(j+1,len(locs_x)):
            p.addCube(objlist[i], 0.05, locs_x[i], locs_y[i], -0.05)
        # p.addBox('table', 0.7, 1.27, 0.54, 0.65, -0.2, -0.38)       
        p.waitForSync()        

        # Move right arm away 
        # stright.pose.position.x = 0.47
        # stright.pose.position.y = -0.54          
        # pos_2 = [1.805870011556845, 0.9850288102385463, -0.5588137158669255, -0.8373051976339934, -0.9004315064597952, 2.0874455972095483, -0.3359333266768987]            
        # gr.moveToPose(stright, "right_gripper", plan_only=False)
        gr.moveToJointPosition(jts_right, rpos2, plan_only=False)

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
        gl.moveToJointPosition(jts_left, lpos2, plan_only=False)
        gl.moveToPose(pickgoal, "left_gripper", plan_only=False)

        pickgoal.pose = rotate_pose_msg_by_euler_angles(pickgoal.pose, 0.0, 0.0, thn)
        gl.moveToPose(pickgoal, "left_gripper", plan_only=False)
        
        pickgoal.pose.position.z = zn
        gl.moveToPose(pickgoal, "left_gripper", max_velocity_scaling_factor = 0.14, plan_only=False)
        leftgripper.close()
        # rospy.sleep(0.25)
        pickgoal.pose.position.z = zn+0.1
        gl.moveToPose(pickgoal, "left_gripper", plan_only=False)
        # gl.moveToJointPosition(jts_left, lpos2, plan_only=False)
        # Move left arm to place object and place object            
        # pos_place = [-1.441426162661994, 0.8389151064712133, 0.14240920034028015, -0.14501001475655606, -1.7630090377446503, -1.5706376573674472, 0.09225918246029519]            
        
        gl.moveToJointPosition(jts_left, lpos1, plan_only=False)
        # gl.moveToPose(stleft, "left_gripper", plan_only=False) 
        stleft = PoseStamped() 
        stleft.header.frame_id = "base"
        stleft.header.stamp = rospy.Time.now()
        stleft.pose.position.x = 0.65
        stleft.pose.position.y = 0.62
        stleft.pose.position.z = 0.0
        stleft.pose.orientation.x = 1.0
        stleft.pose.orientation.y = 0.0
        stleft.pose.orientation.z = 0.0
        stleft.pose.orientation.w = 0.0
        # stleft.pose.position.z = -0.1
        
        # for i in range(len(locs_x)):
        stleft.pose.position.z = -0.10+(j*0.05)
        gl.moveToPose(stleft, "left_gripper", max_velocity_scaling_factor=0.18, plan_only=False)
        leftgripper.open()
        # stleft.pose.position.z = 0.3
        # gl.moveToPose(stleft, "left_gripper", plan_only=False)
        gl.moveToJointPosition(jts_left, lpos1, plan_only=False)           
        
        # Sort based on size of cube
        # if sz < 0.045:
        #     stleft.pose.position.x = 0.5
        #     k += 1
        # else:
        #     stleft.pose.position.x = 0.75
        #     m += 1
        # gl.moveToPose(stleft, "left_gripper", plan_only=False) 

        # Stack cubes based on size
        # if sz < 0.022:
            # stleft.pose.position.z = -0.1+(k*0.025)
        # else:
        
        # stleft.pose.position.z = -0.1
        # gl.moveToPose(stleft, "left_gripper", max_velocity_scaling_factor = 0.21, plan_only=False)
        # leftgripper.open()
        # stleft.pose.position.z = 0.3
        # gl.moveToPose(stleft, "left_gripper", max_velocity_scaling_factor = 0.21, plan_only=False)

    # p.removeAttachedObject('table')


if __name__=='__main__':
    try:
        rospy.init_node('pnp', anonymous=True)
        picknplace()

    except rospy.ROSInterruptException:
        pass