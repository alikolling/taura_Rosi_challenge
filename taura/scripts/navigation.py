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

import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Definindo configuracoes iniciais
yaw = np.pi
state = 0
ox, oy = 0., 0.



class Navigation():
    def __init__(self):
        # Iniciando node
        rospy.init_node('navegationNode', anonymous=False)

        self.sub_data = PointCloud2()
        self.sub_data = []
        self.sub_data = rospy.Subscriber('/sensor/velodyne', PointCloud2, self.getVelodyne)

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
        self.graph()
        self.flame = False

    def animate(i, xs, ys):
        global ox, oy
        # Add x and y to lists
        xs.append(ox)
        ys.append(oy)

        # Draw x and y lists
        ax.clear()
        ax.plot(xs, ys)
        
        if self.flame:
            ax.plot(ox, oy, 'ro', markersize=15, label='Flame')
            self.flame = False

        # Format plot
        plt.title('Odometry')
        plt.ylabel('X(m)')
        plt.xlabel('y(m)')

    def graph():
        global ox, oy
        fig = plt.figure()
        ax = fig.add_subplot(1, 1, 1)
        xs = []
        ys = []

        # Set up plot to call animate() function periodically
        ani = animation.FuncAnimation(fig, animate, fargs=(xs, ys), interval=1000)
        plt.show()

    # Funcao de retorno da odometria
    def getOdometry(self, msg):
        global ox, oy
        ox = msg.pose.pose.position.x
        oy = msg.pose.pose.position.y
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

    # Funcao de retorno do velodyne
    def getVelodyne(self, msg):
        # Recebendo odometria
        teste = pc2.pointcloud2_to_array(msg)
        x = []
        y = []
        z = []

        # Filtrando dados (pontos) para diminuir quantidade de processamento
        for i, j in enumerate(teste['x']):
            if i%5 == 0:
                x.append(j) 

        for i, j in enumerate(teste['y']):
            if i%5 == 0:
                y.append(j)

        for i, j in enumerate(teste['z']):
            if i%5 == 0:
                z.append(j)

        # Transformando valores em um array
        teste_menor = np.array([(i, j, k) for i, j, k in zip(x, y, z)], teste.dtype)

        # Criando dataframe para armazenar os dados
        df = pd.DataFrame(teste_menor, columns=['x','y','z'])

        print('--------------------------------------------------------------------------------------')

        df = df[df['z'] > 0.15]

        # Definindo tolerancias para os filtros
        tolerance = 0.1
        negative_tolerance = -0.1

        # Filtrando menor distancia frontal
        self.frente = np.nan
        distancia_frontal = df[df['y'] < tolerance]
        distancia_frontal = distancia_frontal[distancia_frontal['y'] > negative_tolerance]
        distancia_frontal = distancia_frontal[distancia_frontal['x'] > negative_tolerance]
        print('Distancia frontal: ')
        if(distancia_frontal.isnull().values.all()):
            print('Nada a frente')
            self.frente = np.nan
        else:
            distancia_frontal.reset_index(drop=True, inplace=True)
            value = distancia_frontal[['x']].idxmin()
            distancia_frontal = distancia_frontal.iloc[value][:]
            self.frente = distancia_frontal.at[value[0],'x']
            print(self.frente)
        print('')

        # Filtrando menor distancia da direita (90 graus)
        self.direita = np.nan
        distancia_direita = df[df['x'] < tolerance]
        distancia_direita = distancia_direita[distancia_direita['y'] < 0]
        distancia_direita = distancia_direita[distancia_direita['x'] > negative_tolerance]
        print('Distancia direita: ')
        if(distancia_direita.isnull().values.all()):
            print('Nada a direita')
            self.direita = np.nan
        else:
            distancia_direita.reset_index(drop=True, inplace=True)
            value = distancia_direita[['y']].idxmin()
            distancia_direita = distancia_direita.iloc[value][:]
            self.direita = distancia_direita.at[value[0],'y']
            self.direita = abs(self.direita)
            print(self.direita)
        print('')

        # Filtrando menor distancia da esquerda (90 graus)
        self.esquerda = np.nan
        distancia_esquerda = df[df['x'] < tolerance]
        distancia_esquerda = distancia_esquerda[distancia_esquerda['y'] > 0]
        distancia_esquerda = distancia_esquerda[distancia_esquerda['x'] > negative_tolerance]
        print('Distancia esquerda: ')
        if(distancia_esquerda.isnull().values.all()):
            print('Nada a esquerda')
            self.esquerda = np.nan
        else:
            distancia_esquerda.reset_index(drop=True, inplace=True)
            value = distancia_esquerda[['y']].idxmin()
            distancia_esquerda = distancia_esquerda.iloc[value][:]
            self.esquerda = distancia_esquerda.at[value[0],'y']
            print(self.esquerda)
        print('')

        # Filtrando distancia diagonal direita (45 graus)
        #self.s_direita = np.nan
        dd_direita = df.loc[((abs(df['x'] + df['y'])) <= (tolerance)) & df['x'] > 0]
        print('Distancia diagonal direita: ')
        if(dd_direita.isnull().values.all()):
            print('Nada na diagonal direita')
            #self.s_direita = np.nan
        else:
            dd_direita.reset_index(drop=True, inplace=True)
            value = dd_direita[['x']].idxmin()
            dd_direita = dd_direita.iloc[value][:]
            #print(dd_direita.at[value[0],'x'])
            #x = dd_direita.at[value[0],'x']**2
            self.s_direita = dd_direita.at[value[0],'y']
            self.s_direita = abs(self.s_direita)
            #soma = x + y
            #hip_direita = math.sqrt(soma)
            print(self.s_direita)
        print('')

        # Filtrando distancia diagonal esquerda (45 graus)
        #self.s_direita = np.nan
        dd_esquerda = df.loc[((abs(df['x'] - 2*df['y'])) <= (tolerance)) & df['x'] > 0]
        print('Distancia diagonal esquerda: ')
        if(dd_esquerda.isnull().values.all()):
            print('Nada na diagonal esquerda')
            #self.s_direita = np.nan
        else:
            dd_esquerda.reset_index(drop=True, inplace=True)
            value = dd_esquerda[['x']].idxmin()
            dd_esquerda = dd_esquerda.iloc[value][:]
            #print(dd_esquerda.at[value[0],'x'])
            #x = dd_esquerda.at[value[0],'x']**2
            self.s_esquerda = dd_esquerda.at[value[0],'y']
            #soma = x + y
            #hip_esquerda = math.sqrt(soma)
            #print(hip_esquerda)
            print(self.s_esquerda)
        print('')      

    def giro(self, lado, objetivo):
        # Definindo variaveis globais
        global yaw
        global state

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
        Navigation.andar(self, 1)
        self.flame = True
        Navigation.andar(self, 2.5)
        Navigation.giro(self, -1, 3.14)
        Navigation.andar(self, 1.9)

        # Desviando da caixa grande
        Navigation.giro(self, -1, -2.4)
        Navigation.andar(self, 1.2)
        Navigation.giro(self, 1, -3.0)
        Navigation.andar(self, 1)
        Navigation.giro(self, 1, 2.5)
        Navigation.andar(self, 0.45)
        Navigation.giro(self, -1, -3.13)


        Navigation.andar(self, 2.3)
        Navigation.giro(self, 1, -3.1)
        Navigation.andar(self, 4.5)

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
