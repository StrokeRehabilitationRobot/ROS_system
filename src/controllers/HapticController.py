#!/usr/bin/env python

import sys
import rospy
from strokeRehabSystem.msg import *
from geometry_msgs.msg import Pose,Point, WrenchStamped
import math
import numpy as np
import PDController
import EnviromentDynamics
import tools.helper
import tools.dynamics
import games.maze.maze_helper as maze_helper


class HapticController():

    def __init__(self):
        """
        """
        rospy.init_node("haptics")
        rospy.Subscriber("haptic", hapticForce, self.make_forces)
        K = np.array([[ 0.01, 0],[ 0, 0.001]])
        B = np.array([[ 0.01, 0],[ 0, 0.001]])
        d_goal = 0.5*maze_helper.BLOCKSIZE_X + 0.5*maze_helper.PLAYERSIZE_X + 1.5*maze_helper.BLOCKSIZE_X
        d_obs = 0.5*maze_helper.BLOCKSIZE_X + 0.5*maze_helper.PLAYERSIZE_X + maze_helper.BLOCKSIZE_X
        self.controller = PDController.PDController(K,B)
        self.enviroment =  EnviromentDynamics.EnviromentDynamics(0.01,0.001,0.0001,0.0001,d_obs,d_goal)


    def make_forces(self, haptic):
        """
        subscriber to the haptic force
        gets the joints states from listerener
        pass into the controller and Enviroment
        computers the forces
        """
        (position, velocity, effort) = tools.helper.call_return_joint_states()
        (j1,j2,j3) = tools.dynamics.get_jacobian_matricies(position)
        task_velocity = np.array(j3).dot(np.array(velocity).reshape(3, 1))
        yd = tools.helper.remap(task_velocity[1],-0.20,0.20,0,maze_helper.windowWidth)
        zd = tools.helper.remap(-task_velocity[2],0.10,-0.15,0,maze_helper.windowHeight)
        print [yd,zd]

    def move(self):
        """
        does the position update of a 2nd order system
        pusblish the new position
        """
        pass
if __name__ == '__main__':
    haptic = HapticController()
    rospy.spin()
