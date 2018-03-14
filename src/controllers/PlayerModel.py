#!/usr/bin/env python

import rospy
import numpy as np
import time
from geometry_msgs.msg import Pose,Point, WrenchStamped

class PlayerModel():


    def __init__(self,mass):
        self.mass = mass
        self.state = np.array([[0], [0], [0], [0], [0], [0]])
        self.time0 = time.clock()
        self.pub_player = rospy.Publisher('Player', Point, queue_size=1)


    def move(self, F):
        """
        does the position update of a 2nd order system
        pusblish the new position
        """
        dt = time.clock() - self.time0
        #F = [F.wrench.force.x, F.wrench.force.y, F.wrench.force.z]
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

