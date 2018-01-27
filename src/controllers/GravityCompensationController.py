#!/usr/bin/env python

from tools import dynamics
from tools import Robot
import copy
import numpy as np


class GravityCompensationController():

    def __init__(self, k):
        self._prev = None
        self._K = k
        pass

    def get_torque(self, robot):

        self._prev  = copy.deepcopy(robot)
        g = dynamics.make_gravity_matrix(robot)
        M = dynamics.make_mass_matrix(robot)
        q = np.asarray(robot.q).reshape(3, 1)
        qd = np.asarray(robot.qd).reshape(3, 1)
        load = np.asarray(robot.tau).reshape(3, 1)

        u =  self._K*g

        return u

    def update_K_l(self, k):
        self._K_l = k

    def update_K_v(self, k):
        self._K_v = k
