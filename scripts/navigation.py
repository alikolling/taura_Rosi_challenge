#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2
import pandas as pd
import numpy as np
from ros_numpy import point_cloud2 as pc2
import matplotlib.pyplot as plt

class Navigation():
    def __init__(self):
        rospy.init_node('kkkjj', anonymous=False)

        self.sub_data = PointCloud2()
        self.sub_data = []
        self.sub_data = rospy.Subscriber('/sensor/velodyne', PointCloud2, self.getVelodyne)
        self.pub_velodyne = rospy.Publisher('/sensor/velodyne_menor', PointCloud2, queue_size=10)

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

        teste = pc2.pointcloud2_to_array(msg)
        print(teste.dtype)
        x = []
        y = []
        z = []

        for i, j in enumerate(teste['x']):
            if i%1 == 0:
                x.append(j) 

        for i, j in enumerate(teste['y']):
            if i%1 == 0:
                y.append(j)

        for i, j in enumerate(teste['z']):
            if i%1 == 0:
                z.append(j)

        teste_menor = np.array([(i, j, k) for i, j, k in zip(x, y, z)], teste.dtype)
        ustop = []
        for t in teste_menor:
            #lado esquerdo
            if abs(t['x'])<0.05 and t['y']>0 and t['z']>0:
                ustop.append(t)
            #45 graus lado esquerdo
            if abs(t['x']-t['y'])<0.05 and t['x'] > 0 and t['z'] > 0:
                ustop.append(t)
            #reto 
            if abs(t['y'])<0.05 and t['x'] > 0 and t['z'] > 0:
                ustop.append(t)
            #lado direito
            if abs(t['x'])<0.05 and t['y']<0 and t['z']>0:
                ustop.append(t)
            # 45 graus lado direito
            if abs(t['x']+t['y'])< 0.05 and t['x'] > 0 and t['z'] > 0:
                ustop.append(t)
        
        bls= np.array(ustop, teste.dtype)
        print(bls.shape,'ah n broder')
        
        pub_point = pc2.array_to_pointcloud2(bls, frame_id='velodyne')
        self.pub_velodyne.publish(pub_point)




        #xises = teste['x']
        #t = np.arange(0,len(xises),1)

        #plt.scatter(t, xises)
        #plt.show()
            
        #
        print('aqui')
        print(teste.shape)
        print(teste_menor.shape)


if __name__ == '__main__':
    nav = Navigation()
