#!/usr/bin/env python
import rospy
import numpy as np
import math as m
from geometry_msgs.msg import TwistStamped
from rosi_defy.msg import ManipulatorJoints
from std_msgs.msg import Float64MultiArray
import time


z_torque = 0
distance = 1
height = 0.3
angle = 0
side = 1
theta = [0,0,0,0,0,0]

def move_arm():
	'''
	Funcao que calcula a cinematica inversa para o manipulador UR5

	X eh a distancia entre a base do braco e o cavalete onde sera realizada a medicao
	Z eh a altura onde sera realizada a medicao em relacao ao chao
	rad eh o angulo da ultima junta do manipulador UR5
	side eh o lado onde se encontra o Rosi, 1 para o lado A (esquerda) e 2 para o lado B (direita)
	'''

	# Cria o subscriber arm_coordinates que recebera os comandos e parametros para mover o manipulador
	rospy.Subscriber("arm_coordinates", Float64MultiArray, callback_coordinates)

	global distance
	global height
	global angle
	global side
	global theta

	# Cria os Publishers e os Subscribers que recebem o torque da ponteira de toque e os anguos atuais
	# do manipulador para serem calculados os angulos finais enviados de volta ao UR5
	publisher = rospy.Publisher("/ur5/jointsPosTargetCommand", ManipulatorJoints, queue_size = 1)
	rospy.Subscriber("/ur5/jointsPositionCurrentState", ManipulatorJoints, callback_theta)
	rospy.Subscriber("/ur5/forceTorqueSensorOutput",  TwistStamped, callback_torque)

	# Seta a frequencia do rospy para 3
	rate = rospy.Rate(3)
	while not rospy.is_shutdown():
		# Se o torque eh maior que 0.7, recolhe o UR5 para a posicao inicial
		if z_torque > 0.7:
			distance = 1
			height = 0.3
			angle = 0
			side = 1

		######## Calculos da cinematica inversa para o manipulador UR5 ########
		Y = 0								# Y sempre sera 0, visto que o manipulador sempre estara alinhado ao cavalete na hora da medicao
		X = distance
		Z = height
		rad = angle

		a1 = 0.5							# Distancia do chao para a base do manipulador
		a2 = 0.425							# Tamanho da primeira parte do atuador (entre a base e a junta 2)
		a3 = 0.392							# Tamanho da segunda parte do atuador (entre a junta 2 e a junta 3)

		r = m.sqrt(X**2 + Y**2)
		d = m.sqrt((Z - a1)**2 + r**2)

		# Calculos dos angulos
		if side == 1: theta1 = -np.pi/2
		else: theta1 = np.pi/2
		
		theta3 = m.acos((d**2 - a3**2 - a2**2)/2*a2*a3)
		theta2 = m.atan2(Z - a1, r) - m.atan2(a3*m.sin(theta3), a2 + a3*m.cos(theta3))
		theta4 = -(theta3 + theta2) + rad
		theta5 = -np.pi/2
		theta6 = 0

		final_theta = [theta1,theta2,theta3,theta4,theta5,theta6]


		tolerance = 0.04
		stop = True
		theta = list(theta)
		for a in range(6):
			if abs(theta[a] - final_theta[a]) > tolerance: stop = False
		
		if not stop:
		 	for i in range(6):
		 		if theta[i] < final_theta[i]: theta[i] += 0.02
		 		else: theta[i] -= 0.02

		# VVerifica as condicoes de parada
		tolerance = 0.1

		# Constroi e publica os angulos calculados no topico /ur5/jointsPosTargetCommand
		msg = ManipulatorJoints()
		msg.joint_variable = theta
		
		publisher.publish(msg)
		rate.sleep()

def callback_coordinates(msg):
	'''
	Funcao de callback para o subscriber

	Concatena e retorna os atributos que serao enviados ao calculo dos angulos
	'''
	global distance
	global height
	global angle
	global side

	distance = msg.data[0]
	height = msg.data[1]
	angle = msg.data[2]
	side = msg.data[3]

def callback_theta(msg):
	'''
	Funcao de callback para o subscriber

	Concatena e retorna os angulos atuais das juntas do manipulador UR5
	'''

	global theta
	theta = msg.joint_variable

def callback_torque(msg):
	'''
	Funcao de callback para o subscriber

	Concatena e retorna o torque registrado pela ponteira de toque
	'''
	global z_torque
	z_torque = msg.twist.linear.z

if __name__ == "__main__":
	# Cria o Ros Node e chama a funcao "move_arm"
	rospy.init_node('arm_movement_parameters', anonymous=False)
	try:
		#receive_command()
		move_arm()
	except rospy.ROSInterruptException: pass
