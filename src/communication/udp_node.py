#!/usr/bin/env python
from strokeRehabSystem.srv import *
from strokeRehabSystem.msg import *
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Header
import UDP
import tools.helper
import tools.dynamics
import rospy
import math
import numpy as np

udp = UDP.UDP(9876)
robot_state = rospy.Publisher('joint_states', JointState, queue_size=1)





def udp_callback(downstream):

    q = []
    qd = []
    tau = []
    upstream = udp.send_packet(downstream.board, downstream.id, downstream.packet)
    state = JointState()
    state.header = Header()
    state.header.stamp = rospy.Time.now()

    state.name = ['master_joint0', 'master_joint1', 'master_joint2']

    for i in xrange(3):
        q.append( -round(tools.helper.encoder_to_angle(upstream[i * 3 + 0 + 1]), 2))
        qd.append(round(tools.helper.encoder_to_angle(upstream[i * 3 + 2 + 1]), 2))
        tau.append(tools.helper.filter_tau(upstream[i * 3 + 2 + 1], i))

    q[2]+=0.5*math.pi

    state.position = q
    state.velocity = qd
    state.effort = tau
    robot_state.publish(state)



def torque_callback(force):

    print "hello"
    (position, velocity, effort) = tools.helper.call_return_joint_states()

    if force.header.frame_id == "slave":
        board = 1
    else:
        board = 0

    F = [force.wrench.force.x,force.wrench.force.y,force.wrench.force.z]
    J = tools.dynamics.get_J_tranpose(position)
    tau = np.array(J).dot(np.array(F).reshape(3, 1))
    msg = tools.helper.make_tau_packet(F,1,board)
    print msg
    udp_callback(msg)


def motor_callback(force):

    motor = []
    (position, velocity, effort) = tools.helper.call_return_joint_states()

    if force.header.frame_id == "slave":
        board = 1
    else:
        board = 0

    F = [force.wrench.force.x,force.wrench.force.y,force.wrench.force.z]
    J = tools.dynamics.get_J_tranpose(position)
    tau = np.array(J).dot(np.array(F).reshape(3, 1))
    motor = abs(np.divide(tau, tau))

    packet = tools.helper.make_motor_packet(motor,tau,1,board)

    udp_callback(packet)


def pid_callback(joint):

    if joint.header.frame_id == "slave":
        board = 0
    else:
        board = 1

    packet = tools.helper.make_pid_packet(joint, 0, board)
    udp_callback(packet)


def status_callback(msg):

    packet = tools.helper.make_status_packet()
    udp_callback(packet)


def udp_server():
    rospy.init_node('udp_server')
    rospy.Subscriber("udp", udpMessage, udp_callback)
    rospy.Subscriber("torque_server", WrenchStamped, torque_callback)
    rospy.Subscriber("motors_server", WrenchStamped, motor_callback)
    rospy.Subscriber("pid_server", JointState, pid_callback)
    rospy.Timer(rospy.Duration(0.001), status_callback)
    #udp = UDP.UDP(9876)
    rospy.spin()

if __name__ == "__main__":
    udp_server()
