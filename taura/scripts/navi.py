#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from rosi_defy.msg import RosiMovementArray
from rosi_defy.msg import RosiMovement
import time
import threading

class Navigation():
    def __init__(self):
        # Iniciando node
        rospy.init_node('navegationNode', anonymous=False)

        # Definindo velocidades iniciais
        self.value1 = 0
        self.value2 = 0

        self.pub = rospy.Publisher('/rosi/command_traction_speed', RosiMovementArray, queue_size=10)

        # Criando thread de publicacao nos motores das rodas e no conversor de odometria
        threadPublish = threading.Thread(name = 'publicacao', target = Navigation.publicacao, args = (self,))
        threadPublish.setDaemon(True)
        threadPublish.start()

        # Chamando funcao principal
        Navigation.principal(self)

    # Funcao de publicacao nos motores das rodas e no conversor de odometria
    def publicacao(self):
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

    def andar(self, direita, esquerda, tempo, msg):
        self.value1 = direita
        self.value2 = esquerda
        print(str(msg))
        time.sleep(tempo)
        self.value1 = 0
        self.value2 = 0

    # Funcao principal do programa
    def principal(self):

        Navigation.andar(self, 5, -5, 6, "Estagio1")
        Navigation.andar(self, 5, 5, 26, "Estagio2")
        Navigation.andar(self, -5, 5, 7, "Estagio3")
        Navigation.andar(self, 5, 5, 100, "Estagio4")


if __name__ == '__main__':
    # Inicializando classe Navigation
    nav = Navigation()