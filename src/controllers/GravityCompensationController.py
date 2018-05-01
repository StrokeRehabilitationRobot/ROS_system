#!/usr/bin/env python
"""
    Gravity compensated controller
"""

from tools import dynamics
import numpy as np


class GravityCompensationController:
    """
    Gravity controller
    """

    def __init__(self, k):
        """"
        :param k: gain
        :type np.matrix
        """
        self.K = k

    def get_tau(self, q):
        """
        Calcualte the torque to compensate for gravity

        :param q: joint angles
        :type q: np.array
        :return: force vector (x,y,z)
        :type: np.array
        """

        g = dynamics.make_gravity_matrix(q)
        tau = self.K * g
        u = np.linalg.inv(dynamics.get_J_tranpose(q)) * tau

        return u

    def update_k(self, k):
        """
        changee the gain

        :param k: gain
        :type k: np.matrix
        :return: None
        """
        self.K = k
