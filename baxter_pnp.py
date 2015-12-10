#!/usr/bin/env python

import rospy
import roslib; roslib.load_manifest("moveit_python")
from moveit_python import PlanningSceneInterface, MoveGroupInterface
from geometry_msgs.msg import PoseStamped, PoseArray
import baxter_interface
from moveit_python.geometry import rotate_pose_msg_by_euler_angles
from math import pi, sqrt
from operator import itemgetter

# Function to remove elements in a list at given index
def del_meth(somelist, rem):
    for i in rem:
        somelist[i]='!' 
    for i in range(0,somelist.count('!')):
        somelist.remove('!')
    return somelist

# Pick and place function for baxter (currently supports cubes of different sizes)
def picknplace():
    # Define initial parameters
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
    # pre-determined joint positions for certain poses
    pos1 = [-1.441426162661994, 0.8389151064712133, 0.14240920034028015, -0.14501001475655606, -1.7630090377446503, -1.5706376573674472, 0.09225918246029519,1.7238109084167481, 1.7169079948791506, 0.36930587426147465, -0.33249033539428713, -1.2160632682067871, 1.668587600115967, -1.810097327636719]
    pos2 = [-0.949534106616211, 1.4994662184448244, -0.6036214393432617, -0.7869321432861328, -2.4735440176391603, -1.212228316241455, -0.8690001153442384, 1.8342575250183106, 1.8668546167236328, -0.45674277907104494, -0.21667478604125978, -1.2712865765075685, 1.7472041154052735, -2.4582042097778323]
    lpos1 = [-1.441426162661994, 0.8389151064712133, 0.14240920034028015, -0.14501001475655606, -1.7630090377446503, -1.5706376573674472, 0.09225918246029519]
    lpos2 = [-0.949534106616211, 1.4994662184448244, -0.6036214393432617, -0.7869321432861328, -2.4735440176391603, -1.212228316241455, -0.8690001153442384]    
    rpos1 = [1.7238109084167481, 1.7169079948791506, 0.36930587426147465, -0.33249033539428713, -1.2160632682067871, 1.668587600115967, -1.810097327636719]
    rpos2 = [1.8342575250183106, 1.8668546167236328, -0.45674277907104494, -0.21667478604125978, -1.2712865765075685, 1.7472041154052735, -2.4582042097778323]
    
    # Clear planning scene
    p.clear()
    # Add table as attached object
    p.attachBox('table', 0.7, 1.27, 0.54, 0.65, 0.16, -0.38, 'base', touch_links=['pedestal'])

    # Move both arms to start state              
    g.moveToJointPosition(jts_both, pos1, plan_only=False)
    
    # Get cube locations, filter incorrectly detected cubes & sort cubes based on size
    temp = rospy.wait_for_message("Dpos", PoseArray) # A PoseArray geometry message from baxter_img node is subscribed here
    locs = temp.poses # 4 parameters are obtained: x-y location, orientation and size of cube
    locs_x = []
    locs_y = []
    orien = []
    size = []
    for i in range(len(locs)): # Convert object (cube's center) location to baxter's base frame
        locs_x.append(0.57 + locs[i].position.x)
        locs_y.append(-0.011 + locs[i].position.y)
        orien.append(locs[i].position.z*pi/180)
        size.append(locs[i].orientation.x)
    ind_rmv = []
    for i in range(0,len(locs)): 
        if (locs_y[i] > 0.42):  # Remove objects from list that are in 'place/goal' location
            ind_rmv.append(i)
            continue
        for j in range(i,len(locs)):  # Filter multiple detected locations of same cube/object
            if not (i == j):
                if sqrt((locs_x[i] - locs_x[j])**2 + (locs_y[i] - locs_y[j])**2)<0.01:
                    ind_rmv.append(i)    
    locs_x = del_meth(locs_x, ind_rmv)
    locs_y = del_meth(locs_y, ind_rmv)
    orien = del_meth(orien, ind_rmv) 
    size = del_meth(size, ind_rmv)
    if locs_x:  # Sort list based on cube/object size
        ig0 = itemgetter(0)
        sorted_lists = zip(*sorted(zip(size,locs_x,locs_y,orien), reverse=True, key=ig0))
        locs_x = list(sorted_lists[1])
        locs_y = list(sorted_lists[2])
        orien = list(sorted_lists[3])
        size = list(sorted_lists[0])

    # Loop pick and place until objects are detected on table
    k=0
    while locs_x:
        p.clear()
        p.attachBox('table', 0.7, 1.27, 0.54, 0.65, -0.2, -0.38, 'base', touch_links=['pedestal'])
        p.waitForSync()
        xn = locs_x[0]
        yn = locs_y[0]
        zn = -0.06
        thn = orien[0]
        sz = size[0]
        if thn > pi/4:
            thn = -1*(thn%(pi/4))

        # Add collision objects into planning scene
        objlist = ['obj01', 'obj02', 'obj03', 'obj04', 'obj05', 'obj06', 'obj07', 'obj08', 'obj09', 'obj10', 'obj11']
        for i in range(1,len(locs_x)):
            p.addCube(objlist[i], 0.05, locs_x[i], locs_y[i], -0.05)
        p.waitForSync()        

        # Move both arms to pos2 (Right arm away and left arm on table)
        g.moveToJointPosition(jts_both, pos2, plan_only=False)

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
        gl.moveToPose(pickgoal, "left_gripper", plan_only=False)

        # Orient the left arm correctly
        pickgoal.pose = rotate_pose_msg_by_euler_angles(pickgoal.pose, 0.0, 0.0, thn)
        gl.moveToPose(pickgoal, "left_gripper", plan_only=False) 

        # Lower left arm to pick object with a slow speed
        pickgoal.pose.position.z = zn
        gl.moveToPose(pickgoal, "left_gripper", max_velocity_scaling_factor = 0.14, plan_only=False)
        leftgripper.close()

        pickgoal.pose.position.z = zn+0.1
        gl.moveToPose(pickgoal, "left_gripper", plan_only=False)
        
        # Move both arms to pos1
        g.moveToJointPosition(jts_both, pos1, plan_only=False)        

        # Get NEXT cube locations, filter incorrectly detected cubes & sort cubes based on size
        temp = rospy.wait_for_message("Dpos", PoseArray) # A PoseArray geometry message from baxter_img node is subscribed here
        locs = temp.poses # 4 parameters are obtained: x-y location, orientation and size of cube
        locs_x = []
        locs_y = []
        orien = []
        size = []
        for i in range(len(locs)): # Convert object (cube's center) location to baxter's base frame
            locs_x.append(0.57 + locs[i].position.x)
            locs_y.append(-0.011 + locs[i].position.y)
            orien.append(locs[i].position.z*pi/180)
            size.append(locs[i].orientation.x)
        ind_rmv = []
        for i in range(0,len(locs)): 
            if (locs_y[i] > 0.42): # Remove objects from list that are in 'place/goal' location
                ind_rmv.append(i)
                continue
            for j in range(i,len(locs)): # Filter multiple detected locations of same cube/object
                if not (i == j):
                    if sqrt((locs_x[i] - locs_x[j])**2 + (locs_y[i] - locs_y[j])**2)<0.01:
                        ind_rmv.append(i)    
        locs_x = del_meth(locs_x, ind_rmv)
        locs_y = del_meth(locs_y, ind_rmv)
        orien = del_meth(orien, ind_rmv) 
        size = del_meth(size, ind_rmv)
        if locs_x:  # Sort list based on cube/object size
            ig0 = itemgetter(0)
            sorted_lists = zip(*sorted(zip(size,locs_x,locs_y,orien), reverse=True, key=ig0))
            locs_x = list(sorted_lists[1])
            locs_y = list(sorted_lists[2])
            orien = list(sorted_lists[3])
            size = list(sorted_lists[0])

        # Place object
        stleft = PoseStamped() 
        stleft.header.frame_id = "base"
        stleft.header.stamp = rospy.Time.now()        
       
        # If cube is big, move to pose to stack it up
        if sz > 16.:
            stleft.pose.position.x = 0.65 # was 0.6
            stleft.pose.position.y = 0.7
            stleft.pose.position.z = -0.04+(k*0.05)
            stleft.pose.orientation.x = 1.0
            stleft.pose.orientation.y = 0.0
            stleft.pose.orientation.z = 0.0
            stleft.pose.orientation.w = 0.0
            gl.moveToPose(stleft, "left_gripper", max_velocity_scaling_factor = 0.14, plan_only=False)
            leftgripper.open()
            stleft.pose.position.z = 0.3
            gl.moveToPose(stleft, "left_gripper", plan_only=False)
            gl.moveToJointPosition(jts_left, lpos1, plan_only=False)
            k += 1
        else:
            stleft.pose.position.x = 0.65 # was 0.62
            stleft.pose.position.y = 0.55
            stleft.pose.position.z = 0.2 # was 0.1
            stleft.pose.orientation.x = 1.0
            stleft.pose.orientation.y = 0.0
            stleft.pose.orientation.z = 0.0
            stleft.pose.orientation.w = 0.0
            gl.moveToPose(stleft, "left_gripper", plan_only=False)        
            leftgripper.open()
            gl.moveToJointPosition(jts_left, lpos1, plan_only=False)
        

        # stleft.pose.position.z = 0.3
        # gl.moveToPose(stleft, "left_gripper", plan_only=False)


if __name__=='__main__':
    try:
        rospy.init_node('baxter_pnp', anonymous=True)
        picknplace()

    except rospy.ROSInterruptException:
        pass