#!/usr/bin/env python
"""
    Nathaniel Goldfarb

    This node takes over UDP for send and recieve the commands from the robotic arms
"""

import sys
from strokeRehabSystem.msg import *
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import UDP
import tools.helper
import tools.dynamics
import rospy
import numpy as np

# create a UDP object nad robot publisher
udp = UDP.UDP(9876)
robot_state = rospy.Publisher('unfilter_joint_states', JointState, queue_size=1, latch=True)


def udp_callback(downstream):
    """
    sends and receives messages from the board(s)
    :param downstream: packet to be sent down to the board
    :return:
    """
    q = []
    qd = []
    tau = []
    upstream = udp.send_packet(downstream.board, downstream.id, downstream.packet)
    state = JointState()
    state.header = Header()
    state.header.stamp = rospy.Time.now()

    if downstream.board == 0:
        state.name = ['master_joint0', 'master_joint1', 'master_joint2']
    else:
        state.name = ['slave_joint0', 'slave_joint1', 'slave_joint2']

    for i in xrange(3):
        q.append(-round(tools.helper.encoder_to_angle(upstream[i * 3 + 0 + 1]), 2))
        qd.append(round(tools.helper.encoder_to_angle(upstream[i * 3 + 1 + 1]), 2))
        tau.append(upstream[i * 3 + 2 + 1])

    state.position = q
    state.velocity = qd
    state.effort = tau
    robot_state.publish(state)


def torque_callback(force):
    """
    sends the desired torques to the board. It will turn the nessary motors on/off
    :param force: force to send down to the board
    :type  force: udpTorque.msg
    :return:
    """
    motor = []
    (position, velocity, effort) = tools.helper.call_return_joint_states()

    F = [force.wrench.force.x, force.wrench.force.y, force.wrench.force.z]
    J = tools.dynamics.get_J_tranpose(position)

    tau = np.array(J).dot(np.array(F).reshape(3, 1))
    tau = np.multiply(tau, np.asarray([[-1], [1], [1]]))
    tau = np.round(tau, 4)

    for i in tau:
        if i == 0:
            motor.append(0)
        else:
            motor.append(1)

    packet = tools.helper.make_motor_packet(motor, tau, force.vibration, force.board)

    udp_callback(packet)


def pid_callback(joint):
    """
    takes in a desired joint state
    :param joint:
    :return:
    """

    # TODO implemented this functions
    if joint.header.frame_id == "slave":
        board = 0
    else:
        board = 1

    packet = tools.helper.make_pid_packet(joint, 0, board)
    udp_callback(packet)


def status_callback(msg):
    """
    updates the state of the arms
    :param msg: NULL
    :return:
    """

    packet = tools.helper.make_status_packet()
    udp_callback(packet)
    # packet = tools.helper.make_status_packet(1)
    # udp_callback(packet)


def udp_server():
    """\
    set up the pubs/subs
    """
    rospy.init_node('udp_server')
    rospy.Subscriber("udp", udpMessage, udp_callback)
    rospy.Subscriber("torque_server", udpTorque, torque_callback)
    rospy.Subscriber("pid_server", JointState, pid_callback)
    rospy.Timer(rospy.Duration(0.001), status_callback)
    forces = udpTorque()
    forces.header.frame_id = "master"
    [forces.wrench.force.x, forces.wrench.force.y, forces.wrench.force.z] = [0, 0, 0]
    torque_callback(forces)
    # rospy.Rate(500)
    # udp = UDP.UDP(9876)
    rospy.spin()


if __name__ == "__main__":
    udp_server()
