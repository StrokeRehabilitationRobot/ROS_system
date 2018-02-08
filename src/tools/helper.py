#!/usr/bin/env python
from math import pi
import numpy as np


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

def make_packet(q,qd,tau):
    """

    :param q: joint values
    :param qd: joint vels values
    :param tau: torque vals
    :return:
    """
    packet = 15*[0.0]

    for i in xrange(3):
        packet[3*i] = angle_to_encoder(q[i])
        packet[3*i+1] = qd[i]
        packet[3*i+2] = tau[i]
    packet[6]+= angle_to_encoder(0.5*pi)
    return packet


def norm_tau(u):
    """
    :param u: raw control
    :return: normilized control
    """
    tau = [0,0,0]
    newRange    = [ 0.0  , 2.50 ]
    j1_oldRange = [ 0.001, 0.97 ]
    j2_oldRange = [ 0.001, 0.45 ]
    tau[0] = u[0]#np.interp(u[0], [ -.1, .1   ], [ 0,2.5])
    tau[1] = -u[1]#np.interp(u[1], j1_oldRange, newRange)
    tau[2] = np.interp(u[2], j2_oldRange, newRange)
    return  tau


def remap( x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def filter_tau(interpolated_tau,i):
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
    lengths = [0.25107, 0.191, 0.37843]
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
