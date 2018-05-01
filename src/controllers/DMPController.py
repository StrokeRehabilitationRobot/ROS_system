#!/usr/bin/env python
"""
 Nathaniel Goldfarb

 This is a wrapper class for the DMP controller
"""
import sys
import numpy as np
import lib.dmp.Python.Mod_DMP_runner as dmp


class DMPController(object):
    def __init__(self):
        """
            set up the class varibles
        """
        self.file = None
        self.runner_x = None  # runner for x axis
        self.runner_y = None  # runner for y axis
        self.runner_z = None  # runner for z axis
        self.last_dmp = 3 * [0]  # hold the last state
        self.err = [0, 0, 0]  # holds the error

    def update_dmp_file(self, file_path, start, goal):
        """
        Update what DMP the controller is following

        :param file_path: file path of the DMP
        :type file_path: string
        :param start: staring location
        :type start: np.array
        :param goal: goal location
        :type goal: list
        :return:
        """
        self.file = file_path

        self.runner_x = dmp.Mod_DMP_runner(self.file + "_x.xml", np.asscalar(start[0]), goal[0])
        self.runner_y = dmp.Mod_DMP_runner(self.file + "_y.xml", np.asscalar(start[1]), goal[1])
        self.runner_z = dmp.Mod_DMP_runner(self.file + "_z.xml", np.asscalar(start[2]), goal[2])
        self.last_dmp = 3 * [0]
        self.err = [0, 0, 0]

    def step(self, tau, dt, state, force=np.array([[0], [0], [0]])):
        """

        :param tau: scaling term
        :type tau: DMP scaling term
        :param dt: time step
        :type dt: float
        :param state: current state of the system
        :type state: list
        :param force: externial force
        :type force: np.array
        :return:
        # """
        err_z = abs(state[0] - self.last_dmp[2])[0]
        err_x = abs(state[1] - self.last_dmp[0])[0]
        err_y = abs(state[2] - self.last_dmp[1])[0]
        # self.err = [ err_x, err_y, err_z ]
        alpha = 10
        (x_t, xd_t, xdd_t) = self.runner_x.step(tau, dt, error=alpha * err_x, externail_force=np.asscalar(force[2]))
        (y_t, yd_t, ydd_t) = self.runner_y.step(tau, dt, error=alpha * err_y, externail_force=np.asscalar(force[0]))
        (z_t, zd_t, zdd_t) = self.runner_z.step(tau, dt, error=alpha * err_z, externail_force=np.asscalar(force[1]))

        self.last_dmp[0] = x_t
        self.last_dmp[1] = y_t
        self.last_dmp[2] = z_t
        #
        up = 200 * (np.array([[z_t], [x_t], [y_t]]) - state[0:3])
        uv = 2500 * (np.array([[zd_t], [xd_t], [yd_t]]) - state[3:])
        f = np.array([[zdd_t], [xdd_t], [ydd_t]]) - up - uv

        return f
        # return np.array([[z_t], [x_t], [y_t]])


def dmp_chooser(player, goal):
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
            direction = "right"
        else:
            direction = "left"
    else:
        if sign_y:
            direction = "up"
        else:
            direction = "down"

    return direction
