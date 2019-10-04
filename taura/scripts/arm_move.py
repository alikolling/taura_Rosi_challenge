#!/usr/bin/env python
import rospy
import numpy as np
import math as m
from rosi_defy.msg import ManipulatorJoints
from std_msgs.msg import Float64MultiArray

def main():
	rospy.Subscriber("arm_mover", Float64MultiArray, callback)
	rospy.spin()

def move_arm_to(X,Z,rad = 0, side = 1):
	pub = rospy.Publisher("/ur5/jointsPosTargetCommand", ManipulatorJoints, queue_size = 1)

	#Parameters
	Y = 0

	a1 = 0.5
	a2 = 0.425
	a3 = 0.392

	r = m.sqrt(X**2 + Y**2)
	d = m.sqrt((Z - a1)**2 + r**2)

	#Angulos

	if side == 1: theta1 = -np.pi/2
	else: theta1 = np.pi/2

	theta3 = m.acos((d**2 - a3**2 - a2**2)/2*a2*a3)
	theta2 = m.atan2(r,Z - a1) - m.atan2(a3*m.sin(theta3), a2 + a3*m.cos(theta3))
	theta4 = -(theta3 + theta2) + rad
	theta5 = -np.pi/2
	theta6 = 0

	msg = ManipulatorJoints()
	msg.joint_variable = [theta1, theta2, theta3, theta4, theta5, theta6]
	print(msg)


	pub.publish(msg)
	rospy.spin()

def callback(msg):
	X = msg.data[0]
	Z = msg.data[1]
	rad = msg.data[2]
	side = msg.data[3]

	move_arm_to(X,Z,rad,side)

if __name__ == "__main__":
	rospy.init_node('arm_movement_parameters', anonymous=False)
	try:
		main()
	except rospy.ROSInterruptException: pass
