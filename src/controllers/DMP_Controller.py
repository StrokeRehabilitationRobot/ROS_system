#!/usr/bin/env python

import numpy as np
import strokeRehabSystem.lib.dmp.Python.Mod_DMP_runner as dmp


class DMP_Controller(object):


    def __init__(self, file, start,goal ):

        self.file = file
        self.runner_y = dmp.Mod_DMP_runner(self.file + "_y.xml", start[0], goal[0])
        self.runner_x = dmp.Mod_DMP_runner(self.file + "_x.xml", start[1], goal[1])
        self.runner_z = dmp.Mod_DMP_runner(self.file + "_x.xml", start[1], goal[1])

    def update(self, state, dt, tau=1, err=[0,0,0], force=[0,0,0]):
        """

        :param state: current state of the system
        :param dt: time step
        :param tau: scaling term
        :param err: error between the desired and current
        :param force:
        :return:
        """
        alpha = 0.1
        (x_t, xd_t, xdd_t) = self.runner_x.step(tau, dt, error=alpha*err[0], externail_force=force[0])
        (y_t, yd_t, ydd_t) = self.runner_y.step(tau, dt, error=alpha*err[1], externail_force=force[1])
        (z_t, zd_t, zdd_t) = self.runner_z.step(tau, dt, error=alpha*err[2], externail_force=force[2])

        up = 50 * (np.array([[x_t], [y_t], [z_t]]) - state[0:3])
        uv = 50 * (np.array([[xd_t], [yd_t], [zd_t]]) - state[3:])
        F = np.array([[xdd_t], [ydd_t], [zdd_t]]) - up - uv

        return F