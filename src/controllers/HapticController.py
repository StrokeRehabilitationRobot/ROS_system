#!/usr/bin/env python

import sys
import rospy
from strokeRehabSystem.msg import *
from geometry_msgs.msg import Pose,Point, WrenchStamped
import math
import tf
import numpy as np
import PDController
import EnviromentDynamics
import tools.helper
import tools.dynamics
import games.maze.maze_helper as maze_helper
import time


class HapticController():

    def __init__(self):
        """
        """
        rospy.init_node("haptics")
        rospy.Subscriber("haptic", hapticForce, self.make_forces)
        self.pub_player = rospy.Publisher('Player', Point, queue_size=1)
        K = np.array([[ 400, 0, 0],[ 0,400,0],[ 0,0,400]])
        B = np.array([[ 30, 0, 0],[ 0,30,0],[ 0,0,30]])
        d_goal = 0.5*maze_helper.BLOCKSIZE_X + 0.5*maze_helper.PLAYERSIZE_X + 1.5*maze_helper.BLOCKSIZE_X
        d_obs = 0.5*maze_helper.BLOCKSIZE_X + 0.5*maze_helper.PLAYERSIZE_X + maze_helper.BLOCKSIZE_X
        self.odom_list = tf.TransformListener()
        self.controller = PDController.PDController(K,B)
        self.enviroment =  EnviromentDynamics.EnviromentDynamics(0.01,0.001,0.0001,0.0001,d_obs,d_goal)
        self.state = np.array([[0],[0],[0],[0],[0],[0]])
        self.mass = 1
        self.time0 = time.clock()


    def make_forces(self, haptic):
        """
        subscriber to the haptic force
        gets the joints states from listerener
        pass into the controller and Enviroment
        computers the forces
        """
        (position, velocity, effort) = tools.helper.call_return_joint_states()
        self.odom_list.waitForTransform('base_link', 'master_EE', rospy.Time(0),rospy.Duration(0.1))
        (task_position, _ ) = self.odom_list.lookupTransform('base_link', 'master_EE', rospy.Time(0))
        (j1,j2,j3) = tools.dynamics.get_jacobian_matricies(position)
        task_velocity = np.array(j3).dot(np.array(velocity).reshape(3, 1))
        e =  self.state[0:3]-np.array(task_position).reshape(3, 1)
        #print "position error:", e
        ed =  self.state[3:]-np.array(task_velocity[0:3]).reshape(3, 1)
        F = self.controller.get_F(e,ed)
        F = np.round(F,2)
        self.move(F)

    def move(self, F):
        """
        does the position update of a 2nd order system
        pusblish the new position
        """
        dt = time.clock() - self.time0
        xdd = -np.array(F).reshape(3, 1)/self.mass
        B = np.zeros(shape=(6,3))
        A = np.identity(6)

        B[3,0] = dt
        B[4,1] = dt
        B[5,2] = dt

        A[0,3] = dt
        A[1,4] = dt
        A[2,5] = dt

        self.state = np.dot(A,self.state) + np.dot(B,xdd)
        self.time0 = time.clock()
        player = Point()
        player.x = self.state[1]
        player.y = self.state[2]
        player.z = self.state[0]
        self.pub_player.publish(player)

        #print self.state

if __name__ == '__main__':
    haptic = HapticController()
    rospy.spin()
