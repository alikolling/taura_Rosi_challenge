#!/usr/bin/env python
import rospy
import numpy as np
from rosi_defy.msg import RosiMovement
from rosi_defy.msg import RosiMovementArray
from rosi_defy.msg import ManipulatorJoints

class Rosi():
    
    def __init__(self):
        # sends a message to the user
		rospy.loginfo('Rosi_joy node started')

		# registering to publishers
		self.pub_traction = rospy.Publisher('/rosi/command_traction_speed', RosiMovementArray, queue_size=1)



		# defining the eternal loop frequency
		node_sleep_rate = rospy.Rate(10)

		# eternal loop (until second order)
		while not rospy.is_shutdown():


			traction_command_list = RosiMovementArray()

			# mounting the lists
			for i in range(4):

				# ----- treating the traction commands
				traction_command = RosiMovement()

				# mount traction command list
				traction_command.nodeID = i+1

				# separates each traction side command
				if i < 2:
					traction_command.joint_var = 5
				else:
					traction_command.joint_var = 5

				# appending the command to the list
				traction_command_list.movement_array.append(traction_command)

				# ----- treating the arms commands		


	
			self.pub_traction.publish(traction_command_list)

			# sleeps for a while
			node_sleep_rate.sleep()
        

# instaciate the node
if __name__ == '__main__':

	# initialize the node
	rospy.init_node('rosi_example_node', anonymous=True)

	# instantiate the class
	try:
		node_obj = Rosi()
	except rospy.ROSInterruptException: pass

