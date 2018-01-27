#!/usr/bin/env python

from Dynamics import Dynamics
from Main.Robot import Robot
import copy
import numpy as np


class CompensationController():
    def __init__(self, k_l, k_v, k_g):
        """

        :param k_l:
        :param k_v:
        :param k_g:
        """

        self._K_l = k_l
        self._K_v = k_v
        self._K_g = k_g

    def get_torque(self, robot):
        """

        :param robot:
        :return:
        """

        self._prev  = copy.deepcopy(robot)
        g = dynamics.make_gravity_matrix(robot)
        M = dynamics.make_mass_matrix(robot)
        q = np.asarray(robot.q).reshape(3, 1)
        qd = np.asarray(robot.qd).reshape(3, 1)
        load = np.asarray(robot.tau).reshape(3, 1)

        u =  - ( self._K_v*qd) + (self._K_l*load)   #

        return u

    def update_K_l(self, k):
        self._K_l = k

    def update_K_v(self, k):
        self._K_v = k

    def update_K_g(self, k):
        self._K_g = k
