#!/usr/bin/env python
import sys, time
import rospy
import roslib
import cv2
import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs.msg import PointCloud 
from cv_bridge import CvBridge, CvBridgeError


class Velodyne():
    def __init__(self):
        self.sub = rospy.Subscriber("sensor/velodyne", PointCloud, self.callback, queue_size=1)
        self.bridge = CvBridge()
       
    def callback(self, velodyne):


        #cv_image = self.bridge.imgmsg_to_cv2(velodyne, desired_encoding="passthrough")
        
        print velodyne
        #plt.imshow()
        #plt.show()
        #cv2.imshow('cv_img', cv_image)
        #cv2.waitKey(2)

def main(args):
    '''Initializes and cleanup ros node'''
    vel = Velodyne()
    rospy.init_node('velodyne', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image feature detector module"
    #cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
