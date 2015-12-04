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
    lgrip = baxter_interface.Gripper('left')
    bax = baxter_interface.RobotEnable()
    jts_both = ['left_e0', 'left_e1', 'left_s0', 'left_s1', 'left_w0', 'left_w1', 'left_w2', 'right_e0', 'right_e1', 'right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2']
    jts_right = ['right_e0', 'right_e1', 'right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2']
    jts_left = ['left_e0', 'left_e1', 'left_s0', 'left_s1', 'left_w0', 'left_w1', 'left_w2']

    p.clear()

    # Goto start state    
    pos = [-1.441426162661994, 0.8389151064712133, 0.14240920034028015, -0.14501001475655606, -1.7630090377446503, -1.5706376573674472, 0.09225918246029519, 0.43251610243732586, 0.97134058643113, 0.8794599572168327, -1.3021472716773106, -0.12228190858027599, 1.935400923056818, -2.6711046107800698]
    g.moveToJointPosition(jts_both, pos)

    # Start pick and place    
    D = 100000.
    while not rospy.is_shutdown():
        temp = rospy.wait_for_message("Dpos", PoseArray)
        locs = temp.poses        
        if locs:
            temp = PoseArray()
            locs_base = temp.poses
            # Find nearest object to left arm
            for i in range(len(locs)):
                locs_base[i].position.x = 0.61 + locs[i].position.x
                locs_base[i].position.y = -0.2 + locs[i].position.y
                locs_base[i].position.z = locs[i].position.z*pi/180
                dist = sqrt((locs_base[i].position.x)**2 + (locs_base[i].position.y)**2 + (-0.06)**2)
                if dist < D:
                    D = dist
                    obj_index = i                

            print locs
            print locs_base
            print obj_index

            xn = locs_base[obj_index].position.x
            yn = locs_base[obj_index].position.y
            zn = -0.06
            thn = locs_base[obj_index].position.z

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

            



            ## === Previous code for backup ===





            # xg = 0.7
            # yg = 0.65
            # zg = 0.0

            # placegoal = PoseStamped()
            # placegoal.header.frame_id = "base"
            # placegoal.header.stamp = rospy.Time.now()
            # placegoal.pose.position.x = xg
            # placegoal.pose.position.y = yg
            # placegoal.pose.position.z = zg+0.1
            # placegoal.pose.orientation.x = 1.0
            # g.moveToPose(placegoal, "left_gripper", plan_only=False)

            # placegoal.pose.position.z = zg-0.03
            # g.moveToPose(placegoal, "left_gripper", plan_only=False)

            

            # placegoal.pose.position.z = zg+0.2
            # g.moveToPose(placegoal, "left_gripper", plan_only=False)

        
        else:
            print "No objects detected. Please add objects to pick and place :)"
    
    


    ## === Previous code for backup ===




    # # Need to add initial position for both arms:

    # camview = PoseStamped() # camview represents position of right arm to get view of objects on table

    # camview.header.frame_id = "base"
    # camview.header.stamp = rospy.Time.now()
    # camview.pose.position.x = 0.6
    # camview.pose.position.y = -0.2
    # camview.pose.position.z = 0.6
    # camview.pose.orientation.x = 1.0
    # camview.pose.orientation.y = 0.0
    # camview.pose.orientation.z = 0.0
    # camview.pose.orientation.w = 0.0

    # g.moveToPose(camview, "right_gripper", plan_only=False)

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

    # xn = 0.61+data.poses[0].position.x
    # yn = -0.19+data.poses[0].position.y
    # zn = -0.06

    # leftgripper.calibrate()
    # leftgripper.open()

    # pickgoal = PoseStamped() # Define position for left_arm to move to 

    # pickgoal.header.frame_id = "base"
    # pickgoal.header.stamp = rospy.Time.now()
    # pickgoal.pose.position.x = xn
    # pickgoal.pose.position.y = yn
    # pickgoal.pose.position.z = zn+0.1
    # pickgoal.pose.orientation.x = 1.0  

    # g.moveToPose(pickgoal, "left_gripper", plan_only=False)

    # pickgoal.pose.position.z = zn
    # g.moveToPose(pickgoal, "left_gripper", max_velocity_scaling_factor = 0.1, plan_only=False)
    # leftgripper.close()

    # pickgoal.pose.position.z = zn+0.09
    # g.moveToPose(pickgoal, "left_gripper", max_velocity_scaling_factor = 0.1, plan_only=False)

    # xg = 0.7
    # yg = 0.65
    # zg = 0.0

    # placegoal = PoseStamped()

    # placegoal.header.frame_id = "base"
    # placegoal.header.stamp = rospy.Time.now()
    # placegoal.pose.position.x = xg
    # placegoal.pose.position.y = yg
    # placegoal.pose.position.z = zg+0.1
    # placegoal.pose.orientation.x = 1.0
    # g.moveToPose(placegoal, "left_gripper", plan_only=False)
    
    # placegoal.pose.position.z = zg-0.03
    # g.moveToPose(placegoal, "left_gripper", plan_only=False)

    # leftgripper.open()

    # placegoal.pose.position.z = zg+0.2
    # g.moveToPose(placegoal, "left_gripper", plan_only=False)
    

    # continue for other objects
    # p.removeAttachedObject("table")

# def img_callback():
#     rospy.init_node('img_callback', anonymous=True)
#     rospy.Subscriber("Dpos", PoseArray, picknplace)
#     rospy.spin()

if __name__=='__main__':
    try:
        rospy.init_node('pnp', anonymous=True)
        picknplace()

    except rospy.ROSInterruptException:
        pass