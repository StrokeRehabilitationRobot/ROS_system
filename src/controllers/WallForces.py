import sys
import rospy
from strokeRehabSystem.msg import *
from geometry_msgs.msg import Pose, Point, WrenchStamped
import math
import numpy as np
from math import sin as sin
from math import cos as cos
import games.maze.maze_helper as maze_helper
import tools.dynamics
import tf


class WallForces():

    def __init__(self, k_obs, b_obs, d_obs):
        """
        """
        self.k_obs = -k_obs
        self.b_obs = b_obs
        self.d_obs = d_obs

    def make_force(self, player, enviroment):

        f_y = 0  # force in the y direction (positive DOWN on screen)
        f_x = 0  # force in the x direction (positive RIGHT on screen)
        f_z = 0
        for obs in enviroment.obstacles:

            #print "obs",obs
            #print "player",msg.player
            dx = round(obs.x - player.state[1], 2)
            dy = round(obs.y - player.state[2], 2)
            d = round(math.sqrt(dx ** 2 + dy ** 2),2)
            theta = round(math.atan2(dy, dx),2)

            #print "d", d
            if (max(self.d_obs - d, 0)) != 0:
                F = self.k_obs * (max(self.d_obs - d, 0))
                f_y += -round(F * math.sin(theta), 2) - round(self.b_obs*(player.state[4]),2)
                f_x +=  round(F * math.cos(theta), 2) - round(self.b_obs*(player.state[5]),2)

        if abs(f_y) > abs(f_x):
            f_x = 0
        else:
            f_y = 0
        # Need distance between player and walls
        d = player.state[0] - 0.05
        #print "d ",d
        if (max(0.01 - d, 0)) != 0:
            F = self.k_obs * 1000 * (0.01 - d)
            f_z = round(F, 2) + self.b_obs * (player.state[3])

        d = 0.35 - player.state[0]
        # print "d ",d
        if (max(0.005 - d, 0)) != 0:
            F = 20000 * (0.005 - d)
            f_z = (round(F, 2) + 500 * (player.state[3]))
            #print "f_z", f_z

        f_z = 0

        f_tip = np.asarray([[round(f_z, 1)], [-round(f_x, 1)], [round(f_y, 1)]])

        return f_tip

