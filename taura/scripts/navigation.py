#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
from sensor_msgs.msg import PointCloud2
import pandas as pd
import numpy as np
from ros_numpy import point_cloud2 as pc2
import math
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
from rosi_defy.msg import RosiMovementArray
from rosi_defy.msg import RosiMovement
import time
import threading
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import os
from std_msgs.msg import Float32 

# Definindo configuracoes iniciais
yaw = np.pi
state = 0

class Navigation():
    def __init__(self):
        # Iniciando node
        rospy.init_node('navegationNode', anonymous=False)

        # Subscrevendo no node imu
        self.sub_imu = Imu()
        self.sub_imu = []
        self.sub_imu = rospy.Subscriber('/sensor/imu', Imu, self.getImu)

        # Subscrevendo no node odom_combined
        self.sub_odometry = Odometry()
        self.sub_odometry = []
        self.sub_odometry = rospy.Subscriber('/robot_pose_ekf/odom_combined', PoseWithCovarianceStamped, self.getOdometry)
        
        # Criando publicadores velodyne_menor, command_traction_speed e command_kinect_joint
        self.pub_velodyne = rospy.Publisher('/sensor/velodyne_menor', PointCloud2, queue_size=10)
        self.pub = rospy.Publisher('/rosi/command_traction_speed', RosiMovementArray, queue_size=10)
        self.pubkinect = rospy.Publisher('/rosi/command_kinect_joint', Float32, queue_size=10)

        # Definindo velocidades iniciais
        self.value1 = 0
        self.value2 = 0

        # Criando thread de publicacao nos motores das rodas e no conversor de odometria
        threadPublish = threading.Thread(name = 'publicacao', target = Navigation.publicacao, args = (self,))
        threadPublish.setDaemon(True)
        threadPublish.start()
        time.sleep(10)

        # Criando thread que chamar o roslaunch do rtabmab
        threadMapping = threading.Thread(name = 'mapping', target = Navigation.mapping, args = (self,))
        threadMapping.setDaemon(True)
        threadMapping.start()  
        time.sleep(20)

        # Criando thread da odometria convertida
        threadOdom = threading.Thread(name = 'odometria', target = Navigation.odometria, args = (self,))
        threadOdom.setDaemon(True)
        threadOdom.start()
        time.sleep(5)

        # Chamando funcao principal
        Navigation.principal(self)

    # Funcao de retorno da odometria
    def getOdometry(self, msg):
        self.xPosition = msg.pose.pose.position.x
        self.yPosition = msg.pose.pose.position.y

    # Funcao de publicacao nos motores das rodas e no conversor de odometria
    def publicacao(self):
        # Declarando variaveis 
        self.kinect = Float32()
        self.kinect.data = np.float32(-0.23)

        # Loop de publicacao
        while not rospy.is_shutdown():
            # Iniciando lista de comandos
            self.traction_command_list = RosiMovementArray()

            for i in range(4):
                # Definindo motores
                self.traction_command = RosiMovement()
                self.traction_command.nodeID = i+1

                if i < 2:
                    self.traction_command.joint_var = self.value1
                else:
                    self.traction_command.joint_var = self.value2

                # Salvando os motores na lista de comandos
                self.traction_command_list.movement_array.append(self.traction_command)

            # Publicando na lista de comandos
            self.pub.publish(self.traction_command_list)
            # Publicando no conversor de odometria
            self.pubkinect.publish(self.kinect)

    # Funcao de retorno do imu
    def getImu(self, msg):
        # Definindo variaveis globais
        global yaw
        # Recebendo imu
        data = msg.orientation
        orientation_list = [data.x, data.y, data.z, data.w]
        # Convertendo lista de orientacao da odometria para vetores de euler
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)

    def giro(self, lado, objetivo):
        # Definindo variaveis globais
        global yaw
        global state

        # Definindo estado e tolerancia
        state = 1
        tolerance = 0.1

        # Definindo lado do giro alterando a velocidade angular dos motores
        if(lado == 1):
            self.value1 = -5
            self.value2 = 5
        else:
            self.value1 = 5
            self.value2 = -5

        # Definindo parada no giro
        while(True):
            if(abs(yaw-objetivo) < tolerance):
                state = 0
                break
        
        # Voltando a velocidade dos motores para zero
        self.value1 = 0
        self.value2 = 0

    def andar(self, distancia):
        # Definindo variaveis globais
        global state

        # Definindo estado
        state = 2
        
        # Armazenando posicao inicial
        xi = self.xPosition
        yi = self.yPosition

        # Definindo velocidade algular para frente
        self.value1 = 5
        self.value2 = 5

        
        while(True):
            # Armazenando posicao atual
            xp = self.xPosition
            yp = self.yPosition

            # Calculando distancia ja percorrida entra distancia atual e inicial
            hipotenusa = math.sqrt((xp - xi)**2 + (yp - yi)**2)

            print(hipotenusa)

            # Definindo condicao de parada
            if(hipotenusa > distancia and self.xPosition != 0):
                state = 0
                break
        
        # Voltando a velocidade dos motores para zero
        self.value1 = 0
        self.value2 = 0

    # Funcao principal do programa
    def principal(self):

        # LADO A
        
        # LADO B

        Navigation.giro(self, -1, -3*(np.pi/4))
        Navigation.andar(self, 2.15)
        Navigation.giro(self, 1, -3.0)
        
        Navigation.andar(self, 3)
        Navigation.giro(self, 1, 3.13)
        Navigation.andar(self, 3.5)
        Navigation.giro(self, -1, 3.14)
        Navigation.andar(self, 2.3)

        Navigation.giro(self, -1, -2.5)
        Navigation.andar(self, 1.0)
        Navigation.giro(self, 1, -3.0)
        Navigation.andar(self, 0.75)
        Navigation.giro(self, 1, 2.5)
        Navigation.andar(self, 0.55)
        Navigation.giro(self, -1, -3.13)

        Navigation.andar(self, 2.3)
        Navigation.giro(self, 1, 3.1)
        Navigation.andar(self, 4)

        Navigation.andar(self, 2)
        Navigation.giro(self, 1, 2.9)
        Navigation.andar(self, 7.5)
        Navigation.giro(self, -1, 3.12)
        Navigation.andar(self, 7)

        #Navigation.giro(self, 1, 3.12)
        #Navigation.andar(self, 3)

    # Funcao de inicializacao de mapeamento
    def mapping(self):
        # Chamada do roslaunch para inicializacao do mapping pelo rtabmap
        os.system("roslaunch taura mapping.launch")

    # Funcao de retorno de odometria
    def odometria(self):
        # Chamada do roslaunch para conversao de odometria
        os.system("roslaunch taura robot_pose.launch")

if __name__ == '__main__':
    # Inicializando classe Navigation
    nav = Navigation()
