import time
import threading
import rospy
from std_msgs.msg import Image
from std_msgs.msg import CameraInfo

rospy.init_node('vaiCaralho', anonymous=False)

rgbimage = rospy.Subscriber('/sensor/kinect_rgb', Image, getRGB)
depthimage = rospy.Subscriber('/sensor/kinect_depth', Image, getDepth)
infoimage = rospy.Subscriber('/sensor/kinect_info', CameraInfo, getInfo)

def getRGB(msg):
    global rgb
    rgb = msg.data

def getDepth(msg):
    global depth
    depth = msg.data 

def getInfo(msg):
    global info
    info = msg.data

while not rospy.is_shutdown():
    #rospy.loginfo()
    #pub.publish()
    #rate.sleep()
    print depth