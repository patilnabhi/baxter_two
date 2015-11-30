#!/usr/bin/env python

import rospy
from moveit_python import *
import baxter_interface

def main():

    rospy.init_node("grab_object")
    