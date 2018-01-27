#!/usr/bin/env python
from strokeRehabSystem.srv import *
from strokeRehabSystem.msg import *
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import UDP
import tools.helper
import rospy
import math

udp = UDP.UDP(9876)
robot_state = rospy.Publisher('joint_states', JointState, queue_size=1)


def callback_udp(downstream):
    q = []
    qd = []
    tau = []
    upstream = udp.send_packet(downstream.board, downstream.id, downstream.packet)
    rate = rospy.Rate(0.5) # 10hz
    state = JointState()
    state.header = Header()
    state.header.stamp = rospy.Time.now()
    state.name = ['joint0', 'joint1', 'joint2']
    for i in xrange(3):
        q.append( round(tools.helper.encoder_to_angle(upstream[i * 3 + 0 + 1]), 2))
        q.append(round(tools.helper.encoder_to_angle(upstream[i * 3 + 2 + 1]), 2))
        tau.append(tools.helper.filter_tau(upstream[i * 3 + 2 + 1], i))

    q[2] -= 0.5 * math.pi

    state.position = q
    state.velocity = qd
    state.effort = tau
    robot_state.publish(state)

def udp_server():
    rospy.init_node('udp_server')
    rospy.Subscriber("udp", udpMessage, callback_udp)

    #udp = UDP.UDP(9876)
    rospy.spin()

if __name__ == "__main__":
	udp_server()
