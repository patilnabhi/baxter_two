#!/usr/bin/python

import rospy

# import cv2
# import cv_bridge
from sensor_msgs.msg import Image

class ImageFeed:
    def __init__(self):
        rospy.init_node('display_feed')
        self.sub = rospy.Subscriber('/cameras/right_hand_camera/image', Image, self.callback)
        self.pub = rospy.Publisher('/robot/xdisplay', Image, queue_size=1) #, latch=True
        
    def callback(self,data):
        print ("Publishing feed")
        self.pub.publish(data)

if __name__ == '__main__':
    try:
        imgfeed = ImageFeed()
        rospy.spin()
    except rospy.ROSInterruptException: pass    