#!/usr/bin/env python
import sys, time
import rospy
import roslib
import cv2
import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge, CvBridgeError


class Image_kinect():
    def __init__(self):
        self.sub = rospy.Subscriber("sensor/ur5toolCam", Image, self.callback, queue_size=1)
        self.bridge = CvBridge()
       
    def callback(self, ros_data):


        cv_image = self.bridge.imgmsg_to_cv2(ros_data, desired_encoding="passthrough")
        
        
        #plt.imshow()
        #plt.show()
        cv2.imshow('cv_img', cv_image)
        cv2.waitKey(2)

def main(args):
    '''Initializes and cleanup ros node'''
    ic = Image_kinect()
    rospy.init_node('image', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image feature detector module"
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
