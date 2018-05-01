"""
    nahtaniel Goldfarb

    PD controller
"""
import numpy as np


class PDController:
    def __init__(self, k, b):
        """
        set the gains for the PD controller

        :param k: gain
        :type k: np.matrix
        :param b: gain
        :type b: np.matrix
        """
        self.K = k
        self.B = b

    def get_force(self, e, ed):
        """
        Get the force

        :param e: error in position
        :type e: np.array
        :param ed: error in velocity
        :type ed: np.array
        :return: force
        :type: np.array
        """
        F = self.K.dot(e) + self.B.dot(ed)
        return np.asarray(F)

    def set_k(self, k):
        self.K = k

    def set_b(self, b):
        self.B = b
