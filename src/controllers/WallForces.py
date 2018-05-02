"""
    Nathaniel Goldfarb

    This class handles the wall forces from the enviroment
"""
import sys
from strokeRehabSystem.msg import *
import math
import numpy as np


class WallForces:

    def __init__(self, k_obs, b_obs, d_obs):
        """

        :param k_obs: sping wall constant
        :type k_obs: float
        :param b_obs: dampening wall constant
        :type b_obs: float
        :param d_obs: nominal spring distance
        :type d_obs: float
        """
        self.k_obs = -k_obs
        self.b_obs = b_obs
        self.d_obs = d_obs

    def make_force(self, player, enviroment):
        """

        :param player: player object
        :type player: PlayerModel
        :param enviroment: locations of the walls
        :type enviroment: hapticForce()
        :return: force tip
        :type: np.array
        """

        f_y = 0  # force in the y direction (positive DOWN on screen)
        f_x = 0  # force in the x direction (positive RIGHT on screen)
        f_z = 0
        for obs in enviroment.obstacles:

            # print "obs",obs
            # print "player",msg.player
            dx = round(obs.x - player.state[1], 2)
            dy = round(obs.y - player.state[2], 2)
            d = round(math.sqrt(dx ** 2 + dy ** 2), 2)
            theta = round(math.atan2(dy, dx), 2)

            # print "d", d
            if (max(self.d_obs - d, 0)) != 0:
                f = self.k_obs * (max(self.d_obs - d, 0))
                f_y += -round(f * math.sin(theta), 2) + round(self.b_obs * (player.state[4]), 2)
                f_x += round(f * math.cos(theta), 2) + round(self.b_obs * (player.state[5]), 2)

        if abs(f_y) > abs(f_x):
            f_x = 0
        else:
            f_y = 0
        # Need distance between player and walls
        d = player.state[0] - 0.05

        f_z = 0
        if (max(0.01 - d, 0)) != 0:
            f = self.k_obs * 1000 * (0.01 - d)
            f_z = round(f, 2) + self.b_obs * (player.state[3])



        f_tip = np.asarray([[round(f_z, 1)], [-round(f_x, 1)], [-round(f_y, 1)]])

        return f_tip
