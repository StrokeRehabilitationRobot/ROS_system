#!/usr/bin/env python
from strokeRehabSystem.msg import *
from std_msgs.msg import Header
import rospy



if __name__ == "__main__":
    rospy.init_node('udp_test')
    msg = udpMessage()
    msg.header = Header()
    msg.id = 37
    msg.board = 0
    msg.packet = 15*[400,0,0]
    pub = rospy.Publisher('udp', udpMessage, queue_size=1)
    while 1:
        pub.publish(msg)
        
