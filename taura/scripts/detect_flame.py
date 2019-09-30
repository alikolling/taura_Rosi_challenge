#!/usr/bin/env python

import sys, time
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, NavSatFix
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError

#global variable
localization = NavSatFix()

#init variable
pub_flame = rospy.Publisher("flame", Bool, queue_size=10)

def detection_flame():
    sub_image = rospy.Subscriber("sensor/ur5toolCam", Image, image_callback, queue_size=1)
    sub_gps = rospy.Subscriber("sensor/gps", NavSatFix, gps_callback, queue_size=1)
    rospy.init_node('fureimuru')
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('Shtubg donw')
        cv2.destroyAllWindows()

def gps_callback(data):
    localization = data
    

def image_callback(data):
    
    bridge = CvBridge()
    frame = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)    
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    
    lower = (45,175,190)
    upper = (144,255,255)

    mask = cv2.inRange(hsv, lower, upper)
    mask = cv2.erode(mask, None, iterations=8)
    mask = cv2.dilate(mask, None, iterations=2)

    cv2.imshow('image', frame)
    cv2.imshow('mask', mask)
    cv2.waitKey(3)
    
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
		cv2.CHAIN_APPROX_SIMPLE)

    if len(cnts[1]) > 0:
        print('Tah pegando fogo bicho')
	pub_flame.publish(True)
    else:
        print('Cadeh as chamas juvenal')
	pub_flame.publish(False)


if __name__ == "__main__":
    try:
        detection_flame()
    except rospy.ROSInterruptException:
        pass
