#!/usr/bin/env python

from tools import dynamics
from tools import Robot
import copy
import numpy as np


class GravityCompensationController():

    def __init__(self, k):
        self._prev = None
        self.K = k
        pass

    def get_tau(self, q, load):

        activate_grav = self.moving(load)
        g = dynamics.make_gravity_matrix(q)
        M = dynamics.mass_matrix(q)
        temp = self.K*np.linalg.inv(M) * g
        if activate_grav:
            u = np.linalg.inv(dynamics.get_J_tranpose(q))*temp
        else:
            u = np.array([[0],[0],[0]])
        return u

    def update_K(self, k):
        self.K = k


    def moving(self,load):
        print "load1", load[1]
        return not( load[1] < 0.46 or load[1] > 0.53)
