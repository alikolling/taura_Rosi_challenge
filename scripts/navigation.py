#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist

class Navigation():
    def __init__(self):
        rospy.init_node('cmd_vel', anonymous=False)

        self.sub_data = Twist()
        self.sub_data = []
        self.sub_data = rospy.Subscriber('', Twist, self.getMsg)

        pub = rospy.Publisher('', type, queue_size=10)
        rate = rospy.rate(50) # 50hz

        while not rospy.is_shutdown():
            pub.publish()
            rate.sleep()

    def getMsg(self, msg):
        data = msg.data
        print(data)

if __name__ == '__main__':
    Navigation()