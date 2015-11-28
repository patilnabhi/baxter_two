#!/usr/bin/env python

import rospy
from moveit_python import *

def test_addobj():
    rospy.init_node("addobj")
    p = PlanningSceneInterface("base")

    x = 0.5
    y = 0.2
    z = -0.2

    p.addCube("cube_01", 0.05, x, y, z)
    p.addCube("cube_02", 0.05, x, -y, z)
    p.waitForSync()

    # p.removeCollisionObject("cube_01")

if __name__=='__main__':
    try:
        test_addobj()
    except rospy.ROSInterruptException:
        pass