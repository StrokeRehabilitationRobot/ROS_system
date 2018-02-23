#!/usr/bin/env python
import sys
import rospy
from strokeRehabSystem.msg import *
from geometry_msgs.msg import Pose,Point, WrenchStamped
import math
import numpy as np
import games.maze.maze_helper as maze_helper
import tools.dynamics

class EnviromentDynamics():

    def __init__(self, k_obs, k_goal, b_obs,b_goal, d_obs, d_goal, goal_angle=math.pi/3.0 ):
        """
        """
        self.k_obs  = -k_obs
        self.k_goal = k_goal
        self.b_obs  = b_obs
        self.b_goal = -b_goal
        self.d_obs  = d_obs
        self.d_goal = d_goal
        self.goal_angle = goal_angle

    def make_force(self, msg):
        f_y = 0 #force in the y direction (positive DOWN on screen)
        f_x = 0 #force in the x direction (positive RIGHT on screen)

        # for goal_num, g in enumerate(msg.goals):
        #     #print "g",g
        #     d = math.sqrt( (g.x - player.x)**2 + (g.y - player.y)**2  )
        #     theta_gp = math.atan2( (g.y - player.y),(g.x - player.x) )
        #     if len(goals) == 1:
        #         F = self.k_goal * ( max(self.d_goal - d,0))
        #         f_y += round(F*math.sin(theta_gp),2)
        #         f_x += round(F*math.cos(theta_gp),2)
        #     else:
        #         if goal_num == 0:
        #             continue
        #         theta_gg = math.atan2( (goals[goal_num].y - goals[goal_num - 1].y),(goals[goal_num].x - goals[goal_num - 1].x) )
        #         if abs(theta_gg  - theta_gp) <= self.goal_angle:
        #             F = self.k_goal * ( max(self.d_goal - d,0))
        #             f_y += round(F*math.sin(theta_gp),2) + self.b_goal*v[1]
        #             f_x += round(F*math.cos(theta_gp),2) + self.b_goal*v[0]

        for obs in msg.obstacles:

            d = math.sqrt( (obs.x - msg.player.x)**2 + (obs.y - msg.player.y)**2  )
            theta = math.atan2( (obs.y - msg.player.y),(obs.x - msg.player.x) )
            F = self.k_obs * ( max(self.d_obs - d,0))
            f_y += round(F*math.sin(theta),2) #- self.b_obs*msg.v[1]
            f_x += round(F*math.cos(theta),2) #- self.b_obs*msg.v[0]
            print "f_x:", f_x, "\nf_y:", f_y
            # The x direction on-screen is the -y direction in the base frame
            # The y direction on-screen is the -z direction in the base frame

        (position, _v, _e) = tools.helper.call_return_joint_states()
        rot_mat = tools.dynamics.rotation_matrix(position)
        rot_mat_T = np.transpose(rot_mat)
        #print np.dot(np.array(rot_mat), np.array([0,1,0]).reshape(3,1))
        f_tip = np.dot(np.array(rot_mat_T), np.array([0, f_y, f_x]).reshape(3,1))

        return  f_tip #np.array(([-round(f_x,1), -round(f_y, 1), 0 ])).reshape(3, 1)
