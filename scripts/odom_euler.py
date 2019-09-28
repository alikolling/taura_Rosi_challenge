#!/usr/bin/env python
import time
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
import tf
from tf.transformations import euler_from_quaternion



#R




global xod, yod, thod


xod = 0
yod = 0
thod = 0





#--------------leitura odometria visual---------------


def sub_odom():
    sub = rospy.Subscriber('/rtabmap/odom',Odometry, callback_odom)
    rospy.spin()

def callback_odom(data):
    global xod, yod, thod
    xod = data.pose.pose.position.x
    yod = data.pose.pose.position.y
    q1 = data.pose.pose.orientation.x
    q2 = data.pose.pose.orientation.y
    q3 = data.pose.pose.orientation.z
    q4 = data.pose.pose.orientation.w
    q = (q1, q2, q3, q4)
    e = euler_from_quaternion(q)
    print(xod,yod,e)

rospy.init_node("sub_odom")

rate = rospy.Rate(5)

#pub = rospy.Publisher('/odom' ,Empty, queue_size=10)



time.sleep(1)

if __name__ == '__main__':
    sub_odom()
    
		         
 

     	

