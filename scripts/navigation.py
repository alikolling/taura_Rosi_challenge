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
            pass

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
            if i%5 == 0:
                x.append(j) 

        for i, j in enumerate(teste['y']):
            if i%5 == 0:
                y.append(j)

        for i, j in enumerate(teste['z']):
            if i%5 == 0:
                z.append(j)

        teste_menor = np.array([(i, j, k) for i, j, k in zip(x, y, z)], teste.dtype)

        #pub_point = pc2.array_to_pointcloud2(teste_menor, frame_id='velodyne')
        #self.pub_velodyne.publish(pub_point)

        df = pd.DataFrame(teste_menor, columns=['x','y','z'])

        df = df[df['z'] > 0]
        tolerance = 0.1
        negative_tolerance = -0.1

        distancia_frontal = df[df['y'] < tolerance]
        distancia_frontal = distancia_frontal[distancia_frontal['y'] > 0]
        distancia_frontal = distancia_frontal[distancia_frontal['x'] > 0]
        distancia_frontal.reset_index(drop=True, inplace=True)
        value = distancia_frontal[['x']].idxmin()
        distancia_frontal = distancia_frontal.iloc[value][:]

        distancia_direita = df[df['x'] < tolerance]
        distancia_direita = distancia_direita[distancia_direita['x'] < tolerance]
        distancia_direita = distancia_direita[distancia_direita['y'] < 0]
        distancia_direita = distancia_direita[distancia_direita['x'] > negative_tolerance]
        distancia_direita.reset_index(drop=True, inplace=True)
        value = distancia_direita[['x']].idxmin()
        distancia_direita = distancia_direita.iloc[value][:]

        print(distancia_direita)


        #xises = teste['x']
        #t = np.arange(0,len(xises),1)

        #plt.scatter(t, xises)
        #plt.show()
            
        #
        
        print(distancia_frontal)
        #print(teste.shape)
        #print(teste_menor.shape)
        #print(df)


if __name__ == '__main__':
    nav = Navigation()
