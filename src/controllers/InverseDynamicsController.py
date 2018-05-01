#!/usr/bin/env python
"""
 NOT IMPLEMENTED
"""
from tools import dynamics
from tools import Robot
import copy
import numpy as np


class InverseDynamicsController():
    def __init__(self, k_p,k_v):
        self._prev = None
        self._K_p = k_p
        self._K_v = k_v
        pass

    def get_torque(self, robot, traj ):

        self._prev  = copy.deepcopy(robot)
        G = dynamics.make_gravity_matrix(robot)
        C = dynamics.make_coriolis_matrix(robot)
        M = dynamics.make_mass_matrix(robot)
        q = np.asarray(robot.q).reshape(3, 1)
        qd = np.asarray(robot.qd).reshape(3, 1)
        load = np.asarray(robot.tau).reshape(3, 1)

        # desired values
        qdd_d, qd_d, q_d = traj

        qdd_d = np.asarray(qdd_d).reshape(3, 1)
        qd_d = np.asarray(qd_d).reshape(3, 1)
        q_d = np.asarray(q_d).reshape(3, 1)

        aq = qdd_d - self._K_v*(qd_d - qd) - self._K_p*(q_d - q)


        u = M*aq + C*qd + G + load
        return u

    def update_K_l(self, k):
        self._K_l = k


    def update_K_v(self, k):
        self._K_v = k
