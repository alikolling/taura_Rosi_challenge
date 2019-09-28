#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2
import pandas as pd
import numpy as np

class Navigation():
    def __init__(self):
        rospy.init_node('kkkjj', anonymous=False)

        self.sub_data = PointCloud2()
        self.sub_data = []
        self.sub_data = rospy.Subscriber('/sensor/velodyne', PointCloud2, self.getVelodyne)

        #self.sub_data = Twist()
        #self.sub_data = []
        #self.sub_data = rospy.Subscriber('', Twist, self.getMsg)

        #pub = rospy.Publisher('', type, queue_size=10)
        #rate = rospy.rate(10) # 50hz

        while not rospy.is_shutdown():
            #pub.publish()
            #rate.sleep()
            #print("----")
            x = 0

    def getMsg(self, msg):
        data = msg.data
        print(data)

    def getVelodyne(self, msg):
        cloud_msg = msg
        #print(data)

        dtype_list = [(f.name, np.float32) for f in cloud_msg.fields]
        cloud_arr = np.fromstring(cloud_msg.data, dtype_list)
        teste = np.reshape(cloud_arr, (cloud_msg.height, cloud_msg.width)) 

        print(teste)
        print(teste.shape)

if __name__ == '__main__':
    nav = Navigation()