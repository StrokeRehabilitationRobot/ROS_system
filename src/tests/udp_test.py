#!/usr/bin/env python
from strokeRehabSystem.msg import *
from std_msgs.msg import Header
import rospy



def talker():
    pub = rospy.Publisher('udp', udpMessage, queue_size=1)
    rospy.init_node('udp_test')
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        msg = udpMessage()
        msg.header = Header()
        msg.id = 37
        msg.board = 0
        msg.packet = 5*[400,0,0]
        pub = rospy.Publisher('udp', udpMessage, queue_size=1)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
