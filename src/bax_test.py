#!/usr/bin/python

##Wait until system is completely initialized. Otherwise errors may be encountered

import sys
import copy
import rospy
# import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import moveit_python as mip
from baxter_interface import Limb, Gripper #, RobotEnable, CameraController, DigitalIO
# import rospkg
from os import system

class RobotState:
    def __init__(self):
        pass
        # self._robot=RobotEnable()
        # self._rospack=rospkg.RosPack()
        # self._baxter_tools=rospack.get_path('baxter_tools')
    def enable(self):
            system('python `rospack find baxter_tools`/scripts/enable_robot.py -e')
        # system('python ' + _baxter_tools + '/scripts/enable_robot.py -e')
        # self._robot.enable()  #ditched since it's success is contingent on other processes. It's safer to just incorporate the safeguards of already-existing scripts
    def disable(self):
            system('python `rospack find baxter_tools`/scripts/enable_robot.py -d')
    def reset(self):
            system('python `rospack find baxter_tools`/scripts/enable_robot.py -r')
            # system('python ' + _baxter_tools + '/scripts/enable_robot.py -r')
    def tuck(self):
            system('python `rospack find baxter_tools`/scripts/tuck_arms.py -t')
            # system('python ' + _baxter_tools + '/scripts/tuck_arms.py -t')
    def untuck(self):
            system('python `rospack find baxter_tools`/scripts/tuck_arms.py -u')
            # system(_baxter_tools + '/scripts/tuck_arms.py -u')
    def camera(self,action,cam):
            #accepts "open" or "close" as action, and "head","left" or "right" as cam
            if action == "open":
                a = " -o"
            elif action == "close":
                a= " -c"
            if cam == 'head':
                c=" head_camera"
            elif cam == 'right':
                c=" right_hand_camera"
            elif cam == 'left':
                c=" left_hand_camera"
            system('python `rospack find baxter_tools`/scripts/camera_control.py' +a +c)
            # system(_baxter_tools + '/scripts/camera_control.py' +a +c)
    def smile(self):
        system('python `rospack find baxter_examples`/scripts/xdisplay_image.py --file=`rospack find bax_two`/images/honhon.png')

    def calibrate_gripper(self):
        pass
        ##USE MoveIt instead
# #Display to Baxter's face. Publish: /robot/xdisplay 
# #Cameras: Subscribe: /cameras/head_camera/image
# import cv2
# import cv_bridge
# from sensor_msgs.msg import Image1



# #To enable/disable/reset robot
# import baxter_interface
# from baxter_interface import CHECK_VERSION
# rospy.init_node('rsdk_robot_enable')
# rs = baxter_interface.RobotEnable(CHECK_VERSION)
# #print rs.state() ; rs.enable() ; rs.disable() ; rs.reset()

    
