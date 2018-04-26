#!/usr/bin/env python
import sys
import numpy as np
import lib.dmp.Python.Mod_DMP_runner as dmp



class DMPController(object):


    def __init__(self):

        self.file = None
        self.runner_x = None
        self.runner_y = None
        self.runner_z = None
        self.last_dmp = 3 * [0]
        self.err = [0, 0, 0]

    def update_dmp_file(self,file,start,goal):
        """

        :param file:
        :param start:
        :param goal:
        :return:
        """
        self.file = file
        print self.file + "_x.xml"

        self.runner_x = dmp.Mod_DMP_runner(self.file + "_x.xml", np.asscalar( start[0]), goal[0])
        self.runner_y = dmp.Mod_DMP_runner(self.file + "_y.xml", np.asscalar( start[1]), goal[1])
        self.runner_z = dmp.Mod_DMP_runner(self.file + "_z.xml", np.asscalar( start[2]), goal[2])
        self.last_dmp = 3*[0]
        self.err = [0,0,0]
        print "yoooooooooooooooo"
    def step(self, tau, dt,state, force=[0,0,0]):
        """

        :param state: current state of the system
        :param dt: time step
        :param tau: scaling term
        :param err: error between the desired and current
        :param force:
        :return:
        # """
        err_z = abs(state[0] - self.last_dmp[2])[0]
        err_x = abs(state[1] - self.last_dmp[0])[0]
        err_y = abs(state[2] - self.last_dmp[1])[0]
        print "err", err_z
        # self.err = [ err_x, err_y, err_z ]
        alpha = 0.1
        print force
        (x_t, xd_t, xdd_t) = self.runner_x.step(tau, dt, error=alpha*err_x,externail_force=np.asscalar(force[2]))
        (y_t, yd_t, ydd_t) = self.runner_y.step(tau, dt, error=alpha*err_y, externail_force=np.asscalar(force[0]))
        (z_t, zd_t, zdd_t) = self.runner_z.step(tau, dt, error=alpha*err_z, externail_force=np.asscalar(force[1]))

        self.last_dmp[0] = x_t
        self.last_dmp[1] = y_t
        self.last_dmp[2] = z_t
        #
        up = 100 * (np.array([[z_t], [x_t], [y_t]]) - state[0:3])
        uv = 3000 * (np.array([[zd_t], [xd_t], [yd_t]]) - state[3:])
        F = np.array([[zdd_t], [xdd_t], [ydd_t]]) - up - uv

        return F
        #return np.array([[z_t], [x_t], [y_t]])

def dmp_chooser(player,goal):
    """

    :param player: tuple of player pos
    :param goal: tuple of goal pos
    :return:
    """

    dx = player[0] - goal[0]
    dy = player[1] - goal[1]

    # 0 = right/left, 1 = up/down
    axis = abs(dx) > abs(dy)
    sign_x = np.sign(dx) == 1
    sign_y = np.sign(dy) == 1

    if axis:
        if not sign_x:
            file = "right"
        else:
            file = "left"
    else:
        if sign_y:
            file = "up"
        else:
            file = "down"

    return file

