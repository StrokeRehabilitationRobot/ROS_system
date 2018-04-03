#!/usr/bin/env python

import rospy
import numpy as np
import time
import math
from geometry_msgs.msg import Pose,Point, WrenchStamped

class PlayerModel():


    def __init__(self,mass):
        self.mass = mass
        self.state  = np.array([[0], [0], [0], [0], [0], [0]])
        self.time0 = time.clock()
        self.pub_player = rospy.Publisher('Player', Point, queue_size=1)


    def move(self,F, obs):
        """
        # state: where the player is in task space
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

        old_state = self.state
        self.state = np.dot(A,self.state) + np.dot(B,xdd)

        flag_x = False
        flag_y = False

        for wall in obs:
            dist_x = abs(self.state[0] - wall.x )
            dist_y = abs(self.state[1] - wall.y )
            #print "dist", dist_x
            flag_x = dist_x < 0.1
            flag_y = dist_y < 0.1
            if flag_x:
                break

        self.time0 = time.clock()
        player = Point()
        player.x = self.state[1]
        player.y = self.state[2]
        player.z = self.state[0]
        self.pub_player.publish(player)


    def detect_collision(self,walls):
        # Move the rect
        self.rect.x += dx
        self.rect.y += dy

        # If you collide with a wall, move out based on velocity
        for wall in walls:
            if self.player.rect.colliderect(wall.rect):
                if dx > 0:  # Moving right; Hit the left side of the wall
                    self.rect.right = wall.rect.left
                if dx < 0:  # Moving left; Hit the right side of the wall
                    self.rect.left = wall.rect.right
                if dy > 0:  # Moving down; Hit the top side of the wall
                    self.rect.bottom = wall.rect.top
                if dy < 0:  # Moving up; Hit the bottom side of the wall
                    self.rect.top = wall.rect.bottom
