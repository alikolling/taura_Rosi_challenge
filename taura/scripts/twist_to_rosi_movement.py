#!/usr/bin/env python

import rospy
import numpy as np
from rosi_defy.msg import RosiMovement
from rosi_defy.msg import RosiMovementArray
from geometry_msgs.msg import Twist
global velocity
velocity = Twist()
max_translational_speed = 37
max_rotational_speed = 10
var_lambda = 0.965
wheel_radius = 0.1324
ycir = 0.531


def twist_callback(msg):
    global velocity
    velocity = msg

def compute_kinematicAMatrix(var_lambda, wheel_radius, ycir):
    # kinematic A matrix 
    matrix_A = np.array([[var_lambda*wheel_radius/2, var_lambda*wheel_radius/2],
					    [(var_lambda*wheel_radius)/(2*ycir), -(var_lambda*wheel_radius)/(2*ycir)]])

    return matrix_A

def talker():
    global velocity
    pub_rosi_movement = rospy.Publisher('/rosi/command_traction_speed', RosiMovementArray, queue_size=1)
    sub_twist = rospy.Subscriber('cmd_vel', Twist, twist_callback)
    rospy.init_node('twist_to_rosi_movement')
    rate = rospy.Rate(10)
    kin_matrix_A = compute_kinematicAMatrix(var_lambda, wheel_radius, ycir)
    while not rospy.is_shutdown():
        vel_linear = velocity.linear.x*100
        vel_angular = velocity.angular.z*50
        print('celocidade',velocity)
        b = np.array([[vel_linear],[vel_angular]])
		# finds the joints control
        x = np.linalg.lstsq(kin_matrix_A, b, rcond=-1)[0]
        print(x)
		# query the sides velocities
        omega_right = np.deg2rad(x[0][0])
        omega_left = np.deg2rad(x[1][0])
        print('omegas',omega_right,' ',omega_left)
        traction_command_list = RosiMovementArray()
        
        for i in range(4):
	        # ----- treating the traction commands
	        traction_command = RosiMovement()

	        # mount traction command list
	        traction_command.nodeID = i+1

	        # separates each traction side command
	        if i < 2:
		        traction_command.joint_var = omega_right
                
	        else:
		        traction_command.joint_var = omega_left

	        # appending the command to the list
	        traction_command_list.movement_array.append(traction_command)
            
        #print('traction_list ',traction_command_list)        
        # publishing	
        pub_rosi_movement.publish(traction_command_list)
        
        # sleeps for a while
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
