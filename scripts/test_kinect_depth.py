#!/usr/bin/env python
import sys, time
import rospy
import roslib
import cv2
import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge, CvBridgeError
#from sensor_msgs.msg import Float32

class Image_kinect_depth():
    def __init__(self):
        self.sub = rospy.Subscriber("sensor/kinect_depth", Image, self.callback, queue_size=1)
        
        #self.pub = rospy.Publisher("/rosi/command_kinect_joint", Float32, queue_size=10)
        self.pub = rospy.Publisher("depth/image", Image, queue_size=10)

    def callback(self, ros_data):
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(ros_data, "bgr8")

        cv2.imshow('cv_img', cv_image/2)
        print ("-------------------------------------------------------------------")
        print cv_image
        self.pub.publish(bridge.imgmsg_to_cv2(ros_data, "bgr8"))
        cv2.waitKey(2)

def main(args):
    '''Initializes and cleanup ros node'''
    ic = Image_kinect_depth()
    rospy.init_node('image')
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image feature detector module"
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
