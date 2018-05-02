#!/usr/bin/env python
from math import pi
import numpy as np
import rospy
import sys
from strokeRehabSystem.srv import ReturnJointStates
from strokeRehabSystem.msg import *
import math


def encoder_to_angle(ticks):
    """

    :param ticks: encoder ticks
    :return: angle in rads
    """
    convert_factor = ((1 / 11.44) * (2 * pi / 360))  # converts from tick->rads
    return ticks * convert_factor


def angle_to_encoder(angle):
    """

    :param angle: angle of servo
    :return: encoder ticks(
    """
    convert_factor = ((1 / 11.44) * (2 * pi / 360))  # converts from tick->rads
    return angle / convert_factor


def make_pid_packet(joint, vib=0, board=0):
    """

    :param q: joint values
    :param qd: joint vels values
    :param tau: torque vals
    :return:
    """
    packet_id = 37
    msg = udpMessage()
    packet = 15 * [0.0]

    for i in xrange(3):
        packet[3 * i] = angle_to_encoder(joint.position[i])
        packet[3 * i + 1] = joint.velocity[i]
        packet[3 * i + 2] = joint.effort[i]

    packet[6] += angle_to_encoder(0.5 * pi)
    packet[9] = vib
    msg.packet = packet
    msg.id = packet_id
    msg.board = board
    return msg


def make_status_packet(board=0):
    """

    :return:
    """
    packet = 15 * [0.0]
    msg = udpMessage()
    id = 38
    msg.id = id
    msg.packet = packet
    msg.board = board
    return msg


def make_motor_packet(motors, tau=[0, 0, 0], vib=0, board=0):
    """

    :return:
    """
    packet_id = 10
    msg = udpMessage()
    packet = 15 * [0.0]
    packet[0:3] = motors
    packet[3:6] = tau
    packet[9] = vib
    msg.packet = packet
    msg.id = packet_id
    msg.board = board
    return msg


def make_tau_packet(tau, vib=0, board=0):
    """

    :return:
    """
    packet_id = 39
    msg = udpMessage()
    packet = 15 * [0.0]
    packet[:3] = tau
    packet[9] = vib
    msg.packet = packet
    msg.id = packet_id
    msg.board = board
    return msg


def call_return_joint_states():
    """
    joint server callback, collects the joint angles
    :return: None
    """

    joint_names = ["master_joint0",
                   "master_joint1",
                   "master_joint2"]
    rospy.wait_for_service("return_joint_states")
    try:
        s = rospy.ServiceProxy("return_joint_states", ReturnJointStates)
        resp = s(joint_names)
    except rospy.ServiceException, e:
        print "error when calling return_joint_states: %s" % e
        sys.exit(1)
    for (ind, joint_name) in enumerate(joint_names):
        if (not resp.found[ind]):
            pass
            # print "joint %s not found!"%joint_name
    return resp.position, resp.velocity, resp.effort


def norm_tau(u):
    """
    :param u: raw control
    :return: normilized control
    """
    tau = [0, 0, 0]
    newRange = [0.0, 2.50]
    j2_oldRange = [0.001, 0.45]
    tau[0] = u[0]  # np.interp(u[0], [ -.1, .1   ], [ 0,2.5])
    tau[1] = -u[1]  # np.interp(u[1], j1_oldRange, newRange)
    tau[2] = np.interp(u[2], j2_oldRange, newRange)
    return tau


def remap(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def filter_tau(interpolated_tau, i):
    # TODO figure out analog way of interpring torque
    max_tau = [0.6, 0.55, 0.55]
    min_tau = [.3, .40, .4]

    if interpolated_tau > max_tau[i]:
        return -1
    elif interpolated_tau < min_tau[i]:
        return 1
    else:
        return 0


def get_lengths():
    lengths = [0.15, 0.21, 0.16]
    return lengths


def get_ineria():
    inertia = [[0.006757, 0.0006036, 0.0015514],
               [0.001745, 0.0005596, 0.00006455],
               [0.00706657, 0.0006254, 0.0015708]
               ]
    return inertia


def get_mass():
    mass = [1.01992, 0.3519, 0.22772]
    return mass


def get_centriod():
    centroid = [0.10424, 0.14550, 0.203]
    return centroid


def distance(p0, p1):
    return math.sqrt((p0[0] - p1[0]) ** 2 + (p0[1] - p1[1]) ** 2)
