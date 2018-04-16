#!/usr/bin/env python

import rospy
import numpy as np
import time
import math
import games.maze.maze_helper as maze_helper
from geometry_msgs.msg import Pose,Point, WrenchStamped
from strokeRehabSystem.msg import *

class PlayerModel():


    def __init__(self,mass,pose):
        self.mass = mass
        self.state  = np.array([[pose[0]], [pose[1]], [pose[2]], [0], [0], [0]])
        self.time0 = time.clock()
        self.pub_player_state = rospy.Publisher('Player_State', PlayerState, queue_size=1)


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
        self.detect_collision(obs)

        msg = PlayerState()

        msg.pose.position.x = self.state[1]
        msg.pose.position.y = self.state[2]
        msg.pose.position.z = self.state[0]

        msg.vel.linear.x = self.state[5]
        msg.vel.linear.y = self.state[4]
        msg.vel.linear.z = self.state[3]

        msg.accel.linear.x = xdd[1]
        msg.accel.linear.y = xdd[2]
        msg.accel.linear.z = xdd[0]

        self.time0 = time.clock()
        self.pub_player_state.publish(msg)

    def detect_collision(self, obstacles):
        # Move the rect

        count = 0
        (px,py) = maze_helper.task_to_game(self.state[1], self.state[2] )
        player = maze_helper.player_to_rect( px , py )
        # If you collide with a wall, move out based on velocity
        wall_game = []

        for obs in obstacles:
            wall_game.append(maze_helper.task_to_game(obs.x,obs.y) )

        walls = map(maze_helper.point_to_rect, wall_game)
        print "walls", wall_game
        fixed_x = False
        fixed_y = False

        for wall in walls:

            if player.colliderect(wall):


                if player.centerx > wall.centerx and player.centery ==  wall.centery:
                    x, y = maze_helper.game_to_task(wall.right+maze_helper.PLAYERSIZE_X, 0 )
                    self.state[1] = x #+ 0.005
                    self.state[4] = 0
                    print "left"

                if player.centerx < wall.centerx and player.centery == wall.centery:
                    x, y = maze_helper.game_to_task(wall.left - maze_helper.PLAYERSIZE_X, 0)
                    self.state[1] = x  # + 0.005
                    self.state[4] = 0
                    print "right"

                if player.centerx == wall.centerx and player.centery > wall.centery:
                    x, y = maze_helper.game_to_task(0,wall.bottom + maze_helper.PLAYERSIZE_Y)
                    self.state[2] = y
                    self.state[5] = 0
                    print "bottom"

                if player.centerx == wall.centerx and player.centery < wall.centery:
                    x, y = maze_helper.game_to_task(0,wall.top - maze_helper.PLAYERSIZE_Y)
                    self.state[2] = y
                    self.state[5] = 0
                    print "top"



        print count
        print "_______________________________________"
