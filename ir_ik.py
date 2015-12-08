#!/usr/bin/env python

# import sys
# import rospy
# from moveit_python import *
# from geometry_msgs.msg import PoseStamped
# import moveit_msgs.msg
# from moveit_msgs.srv import GetPositionIK
# import baxter_interface
# import moveit_commander


import sys
import time
import rospy
import baxter_interface
import tf
import numpy
# from baxter_interface import CHECK_VERSION
from moveit_python import PlanningSceneInterface, MoveGroupInterface

from geometry_msgs.msg import PoseStamped
import moveit_msgs.msg
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import Range

from baxter_pykdl import *

#given robot arm side "right" or "left", returns IR sensor's range reading (+X direction of robot/"side"_hand_range frame)
def getIR(side):
    IR=rospy.wait_for_message("/robot/range/"+side+"_hand_range/state", Range, timeout=None)
    return IR.range

#given string "right" or "left, x distance , and tf instance, returns corresponding angles and displacement in base frame 
def transIRtoBase(side,IRrange,tt):
    t=tt.getLatestCommonTime(side+"_hand_range", "base")
    (trans,orien) = tt.lookupTransform("base",side+"_hand_range",t)  #orn is presumably quaternion
    matrix=tf.transformations.compose_matrix(angles=tf.transformations.euler_from_quaternion(orien), translate=trans)
    pt=tf.transformations.compose_matrix(translate=(IRrange,0,0))
    # numpy.array(((IRrange),(0),(0),(1)))
    ptbase=tf.transformations.decompose_matrix(numpy.dot(matrix,pt))
    euler= ptbase[2]
    translation = ptbase[3]
    quaternion=tf.transformations.quaternion_from_euler(euler[0],euler[1],euler[2])
    return (euler,translation) #orientation in Euler angles; position in x,y,z


#given side "right" or "left" and request, returns list of arm joint angles
def IK_client(side,pose_list):
    # print "Waiting for GetPositionIK service" 
    # rospy.wait_for_service('GetPositionIK')
    try:
        joints = None
        while (joints==None):
            ik= baxter_kinematics(side)
            # d_x=desired_pose.pose.position.x
            # d_y=desired_pose.pose.position.y
            # d_z=desired_pose.pose.position.z
            # d_ox=desired_pose.pose.orientation.x
            # d_oy=desired_pose.pose.orientation.y
            # d_oz=desired_pose.pose.orientation.z
            # d_ow=desired_pose.pose.orientation.w
            print "Calling IK service"
            joints = ik.inverse_kinematics(pose_list[0:3], pose_list[3:7])
            # getJoints = rospy.ServiceProxy('compute_ik', GetPositionIK)
            # # sol = getJoints(request)
            # if side == "left":
            #     joints=sol.solution.joint_state.position[1:8]
            # elif side == "right":
            #     joints=sol.solution.joint_state.position[8:15]
            print joints

        return joints
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


#given side "left" or "right", list of current joint angles in desired arm, desired PoseStamped, returns request for MoveIt! IK joint solution
def initRequest(side,desired_pose,current_joints=[],timeout=2):
    print "Generating Request"
    request=moveit_msgs.msg.PositionIKRequest()
    request.group_name=side+"_arm"
    if len(current_joints)==7:
        cjoints = current_joints
    elif side=="right":
        print "Using right arm neutral position as reference for IK"
        cjoints = [1.2, 2.0, 0.1, -1.0, -0.67, 1.0, 0.5]
    elif side =="left":
        print "Using left arm neutral position as reference for IK"
        cjoints =[-1.2, 2.0, -0.1, -1.0, 0.67, 1.0, -0.5]
    request.robot_state.joint_state.position=cjoints
    request.robot_state.joint_state.name=[side+'_e0', side+'_e1', side+'_s0', side+'_s1', side+'_w0', side+'_w1', side+'_w2']
    request.avoid_collisions=True
    # request.ik_link_names=robot.get_link_names(group = robot.get_group("left_arm"))
    request.pose_stamped=desired_pose
    request.timeout=rospy.Duration.from_sec(timeout)
    request.attempts=100
    print "Request created"
    return request

def initPose(px,py,pz,ox=1.0,oy=0.0,oz=0.0,ow=0.0):
    print "Creating PoseStamped"
    Pose=PoseStamped()
    Pose.header.stamp = rospy.Time.now()
    Pose.pose.position.x = px
    Pose.pose.position.y = py
    Pose.pose.position.z = pz
    Pose.pose.orientation.x = ox
    Pose.pose.orientation.y = oy
    Pose.pose.orientation.z = oz
    Pose.pose.orientation.w = ow
    print "PoseStamped Created"
    return Pose

#if any new joint position is farther than 3/2 Pi from original position, return False
def JointTest(curr_joints,new_joints):
    print "Testing joint solution"
    for i in range(len(new_joints)):
        if (abs(numpy.copy(curr_joints[i])-numpy.copy(new_joints[i])))>(3.14*1.5):
            print curr_joints[i]-new_joints[i]
            return False
    print max(curr_joints-new_joints)
    return True


#given side "right" or "left", list of current joint positions of desired arm, desired pose, returns suitable joint states for desired pose
#optionally give test=True if want to try filtering crazy joint motions. Untested; may give identical results every time...
#currently doesn't take into account external collision objects. Come on Mike Ferguson! You can do it!
def transJoints(side,curr_joints,pose,test=False,iterations=5):
    print "Calling request function"
    # request = initRequest(side,pose,current_joints=curr_joints,timeout=2)
    new_joints=IK_client(side,pose)

    if test:
        satisfaction=JointTest(curr_joints,new_joints)
        count = 0
        while (not satisfaction) and (count<iterations):
            new_joints=IK_client(side,pose)
            satisfaction=JointTest(curr_joints,new_joints)
            count += 1
    print "Returning joint solution"
    print "This testing took %i iterations" %(count)
    return new_joints           

def main_IKsolverDemo():
    xn = 1.0
    yn = 1.0
    zn = 0.0

    # pickgoal=initPose(xn,yn,zn,ox=1.0,oy=0.0,oz=0.0,ow=0.0)
    pickgoal=[0.75,0.6,0.25,1.0,0.0,0.0,0.0]
    initial_joints=[-1.2, 2.0, -0.1, -1.0, 0.67, 1.0, -0.5]
    solution=transJoints("left",initial_joints,pickgoal,test=True,iterations=150)
    print "Hurray"
    print solution


def main_IRrangeDemo(side):
    tt = tf.TransformListener()
    rospy.sleep(1)
    rate = rospy.Rate(2)

    print "Starting IRrange Loop"
    while not rospy.is_shutdown():
        try:
            IRrange=getIR(side)
            if IRrange>65 or IRrange<0.15:
                print "IR sensor reading is outside range: %.1f" %(IRrange)
                rate.sleep()
                continue
            (euler,translation)=transIRtoBase(side,IRrange,tt)
            print "Current reading: %f corresponds to base frame z= %f" %(IRrange,translation[2])
            rate.sleep()

        except rospy.ROSInterruptException, e:
            print "ROSInterruptException: %s"%e
            print "Have a nice day."





rospy.init_node('helper_demo')
rospy.sleep(1)
print "Initializing Robot"
# rs = baxter_interface.RobotEnable(CHECK_VERSION)
# if not rs.state().enabled:
#     rs.enable()
# rospy.on_shutdown(clean_shutdown(init_state, rs))
main_IKsolverDemo()
# main_IRrangeDemo("left")

# def clean_shutdown(init_state, rs):
#     if not init_state:
#         print("Disabling robot...")
#         rs.disable()