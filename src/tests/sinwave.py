#!/usr/bin/env python
from strokeRehabSystem.msg import *
from std_msgs.msg import Header
import rospy



# def three_sin():
#     viaPts = [0, -400, 400, -400, 400, 0];
#     packet = 15*[0]
#     pub = rospy.Publisher('udp', udpMessage, queue_size=1)
#     rospy.init_node('udp_test')
#     #rate = rospy.Rate(10) # 10hz
#     for via in viaPts:
#         msg = udpMessage()
#         msg.header = Header()
#         msg.id = 37
#         msg.board = 0
#         packet[0] = via
#         msg.packet = packet
#         pub = rospy.Publisher('udp', udpMessage, queue_size=1)
#         pub.publish(msg)
#         rospy.sleep(0.5)
#
#     for via in viaPts:
#         msg = udpMessage()
#         msg.header = Header()
#         msg.id = 37
#         msg.board = 0
#         packet[3] = via
#         msg.packet = packet
#         pub = rospy.Publisher('udp', udpMessage, queue_size=1)
#         pub.publish(msg)
#         rospy.sleep(0.5)
#
#     for via in viaPts:
#         msg = udpMessage()
#         msg.header = Header()
#         msg.id = 37
#         msg.board = 0
#         packet[6] = via
#         msg.packet = packet
#         pub = rospy.Publisher('udp', udpMessage, queue_size=1)
#         pub.publish(msg)
#         rospy.sleep(0.5)


#
# def three_joints():
#     viaPts = [0, -400, 400, -400, 400, 0];
#     packet = 15*[0]
#     pub = rospy.Publisher('udp', udpMessage, queue_size=1)
#     rospy.init_node('udp_test')
#     msg = udpMessage()
#     msg.header = Header()
#     msg.id = 37
#     msg.board = 0
#     for via in viaPts:
#         msg = udpMessage()
#         msg.header = Header()
#         msg.id = 37
#         msg.board = 0
#         packet[0] = via
#         packet[6] = via
#         packet[3] = via
#         msg.packet = packet
#         pub = rospy.Publisher('udp', udpMessage, queue_size=1)
#         pub.publish(msg)
#         rospy.sleep(0.5)

def convert(deg):
    temp = (4096.0/360.0)*deg
    print temp
    return temp

def go_up():
    packet = 15*[0]
    pub = rospy.Publisher('udp', udpMessage, queue_size=1)
    rospy.init_node('udp_test')
    rospy.sleep(5)
    msg = udpMessage()
    msg.header = Header()
    msg.id = 37
    msg.board = 0
    packet[3] = 400#convert(90)
    packet[6] = 400#convert(90)
    pub.publish(msg)
    rospy.sleep(5)



if __name__ == '__main__':
    try:
        #three_sin()
        #three_joints()
        go_up()
    except rospy.ROSInterruptException:
        pass
