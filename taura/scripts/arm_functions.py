import rospy
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped
from rosi_defy.msg import RosiMovement
from rosi_defy.msg import RosiMovementArray
import time

z_torque = 0	# Variavel para controle do torque da ponteira


def touch_trestle(side):
	'''
	Funcao que publica no topico /arm_coordinates os parametros para o UR5 encostar no cavalete
	'''
	# Inicia o no
	rospy.init_node("arm_commander")

	# Se inscreve no topico /ur5/forceTorqueSensorOutput, que retorna o torque da ponteira
	sub = rospy.Subscriber("/ur5/forceTorqueSensorOutput",  TwistStamped, callback_torque)

	# Cria o publisher para publicar as coordenadas ao UR5
	pub = rospy.Publisher("/arm_coordinates", Float64MultiArray, queue_size = 1)

	#### Parametros ####
	distance = 2		# Distancia entre da base do Rosi para a ponteira de toque
	height = 1			# Altura da ponteira de toque em relacao ao chao
	angle = 0			# Angulo com que a ponteira de toque se move em relacao ao chao

	# Cria a mensagem que sera enviada ao braco
	msg = Float64MultiArray()
	msg.data = [height, distance, angle, side]

	timeout = time.time()		# Tempo de timeout, caso haja algum erro
	while z_torque < 0.7 and not rospy.is_shutdown():
		# Enquanto a ponteira de toque nao escostar em nada, continua movendo o manipulador para frente
		# Caso a ponteira fique parada por algum tempo, recolhe o UR5 (significa que o UR5 errou o toque)
		if (time.time() - timeout) > 45: 
			msg.data = [1, 0.3, 0, 1]

		if (time.time() - timeout) > 50: 
			break
		pub.publish(msg)

	sub.unregister()	# Se desisnscreve do topico

def touch_roll(side):
	'''
	Funcao que publica no topico /arm_coordinates os parametros para o braco encostar no cavalete
	'''
	# Inicia o no
	rospy.init_node("arm_commander")

	# Se inscreve no topico /ur5/forceTorqueSensorOutput, que retorna o torque da ponteira
	sub = rospy.Subscriber("/ur5/forceTorqueSensorOutput",  TwistStamped, callback_torque)

	# Cria o publisher pub para publicar as coordenadas ao UR5
	# e o publisher wheel_pub para publicar a velociade de rotacao das esteiras das rodas
	pub = rospy.Publisher("/arm_coordinates", Float64MultiArray, queue_size = 1)
	wheel_pub = rospy.Publisher("/rosi/command_arms_speed", RosiMovementArray, queue_size = 5)

	#### Parametros ####
	distance = 3
	height = 2
	angle = 0.78539815

	# Cria a mensagem que sera enviada ao UR5
	msg = Float64MultiArray()
	msg.data = [height, distance, angle, side]

	# O manipulador UR5 por si so nao alcanca o rolo
	# Para que o toque seja realizado eh necessario abaixar as esteiras das rodas para levantar o Rosi
	#### Inicia o loop para levantar o Rosi ####
	init_time = time.time()
	while not rospy.is_shutdown():
		# Cria a mensagem para rotacionar as esteiras das rodas
		wheel_msg = RosiMovementArray()

		for i in range(4):
			wheel_command = RosiMovement()
			wheel_command.nodeID = i+1

			if i == 0 or i == 2:
				if time.time() - init_time < 23.5:
					wheel_command.joint_var = 2.5
				else:
					wheel_command.joint_var = 0

			if i == 1 or i == 3:
				if time.time() - init_time < 23.5:
					wheel_command.joint_var = -2.7
				else:
					wheel_command.joint_var = 0

			wheel_msg.movement_array.append(wheel_command)
		
		if time.time() - init_time > 25: break
		wheel_pub.publish(wheel_msg)

	#### Movimenta o UR5 ####
	timeout = time.time()
	while z_torque < 0.7 and not rospy.is_shutdown():
		if (time.time() - timeout) > 75: 
			msg.data = [1, 0.3, 0, 1]

		if (time.time() - timeout) > 80: 
			break
		pub.publish(msg)

	sub.unregister()	# Se desregistra dos topicos

	time.sleep(20)

	#### Inicia o loop para abaixar o Rosi ####
	init_time = time.time()
	while not rospy.is_shutdown():
		wheel_msg = RosiMovementArray()

		for i in range(4):
			wheel_command = RosiMovement()
			wheel_command.nodeID = i+1

			if i == 0 or i == 2:
				if time.time() - init_time < 10:
					wheel_command.joint_var = -1.3
				else:
					wheel_command.joint_var = 0

			if i == 1 or i == 3:
				if time.time() - init_time < 9:
					wheel_command.joint_var = 1.3
				else:
					wheel_command.joint_var = 0

			wheel_msg.movement_array.append(wheel_command)
		
		if time.time() - init_time > 12: break
		wheel_pub.publish(wheel_msg)


def callback_torque(msg):
	'''
	Funcao de callback para o Subscriber

	Recebe e retorna o valor do torque da ponteira de toque
	'''
	global z_torque
	z_torque = msg.twist.linear.z


