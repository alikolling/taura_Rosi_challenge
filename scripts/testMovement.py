#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from rosi_defy.msg import RosiMovementArray
from rosi_defy.msg import RosiMovement
import time
import threading
from getkey import getkey, keys

class RosiNode():
    def __init__(self):
        rospy.init_node('rosi_node')

        print(" W - Andar para frente \n S - Andar para trás \n A - Girar para a esquerda \n D - Girar para a direita \n SPACE - Parar \n J - Levantar pés \n K - Baixar pés \n L - Parar pés " )

        pub = rospy.Publisher('/rosi/command_traction_speed', RosiMovementArray, queue_size=10)
        pub2 = rospy.Publisher('/rosi/command_arms_speed', RosiMovementArray, queue_size=10)
        rate = rospy.Rate(10) # 50hz

        blinkLoop = threading.Thread(name = 'manualControl', target = RosiNode.manualControl, args = (self,))
        blinkLoop.setDaemon(True)
        blinkLoop.start()

        self.value1 = 0
        self.value2 = 0

        self.value3 = 0
        self.value4 = 0
        self.shut = False
        rospy.on_shutdown(self.shutdown)


        while not rospy.is_shutdown():     

            self.traction_command_list = RosiMovementArray()
            self.arms_command_list = RosiMovementArray()

            for i in range(4):
                self.traction_command = RosiMovement()
                self.traction_command.nodeID = i+1

                self.arms_command = RosiMovement()
                self.arms_command.nodeID = i+1

                if i < 2:
                    self.traction_command.joint_var = self.value1
                    self.arms_command.joint_var = self.value3
                else:
                    self.traction_command.joint_var = self.value2
                    self.arms_command.joint_var = self.value4

                self.traction_command_list.movement_array.append(self.traction_command)
                self.arms_command_list.movement_array.append(self.arms_command)

            #rospy.loginfo(self.traction_command_list)
            pub.publish(self.traction_command_list)
            pub2.publish(self.arms_command_list)
            rate.sleep()

    def manualControl(self):
        while(True):
            key = getkey()
            if key == 'w':
                self.value1 = 25
                self.value2 = 25
            elif key == 's':
                self.value1 = -15
                self.value2 = -15
            elif key == 'd':
                self.value1 = -25
                self.value2 = 25
            elif key == 'a':
                self.value1 = 25
                self.value2 = -25
            elif key == ' ':
                self.value1 = 0
                self.value2 = 0
            elif key == 'j':
                self.value3 = 10
                self.value4 = 10
            elif key == 'k':
                self.value3 = -10
                self.value4 = -10
            elif key == 'l':
                self.value3 = 0
                self.value4 = 0
            if self.shut:
                print('fechou')
                break

    def shutdown(self):
        print('vai fechar')
        self.shut = True
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        RosiNode()
    except rospy.ROSInterruptException:
        pass
