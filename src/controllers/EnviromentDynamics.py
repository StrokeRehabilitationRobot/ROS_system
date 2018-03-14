#!/usr/bin/env python
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


class EnviromentDynamics():

    def __init__(self, k_obs, k_goal, b_obs, b_goal, d_obs, d_goal, goal_angle=math.pi / 3.0):
        """
        """
        self.k_obs = -k_obs
        self.k_goal = k_goal
        self.b_obs = b_obs
        self.b_goal = -b_goal
        self.d_obs = d_obs
        self.d_goal = d_goal
        self.goal_angle = goal_angle
        self.odom_list = tf.TransformListener()
        self.pub_base = rospy.Publisher('base_force', WrenchStamped, queue_size=1)
        self.pub_tip = rospy.Publisher('tip_force', WrenchStamped, queue_size=1)

    def make_force(self, msg):

        f_y = 0  # force in the y direction (positive DOWN on screen)
        f_x = 0  # force in the x direction (positive RIGHT on screen)
        print "------------------------"
        for obs in msg.obstacles:

            print "obs",obs
            print "player",msg.player
            d = math.sqrt((obs.x - msg.player.x)**2 + (obs.y - msg.player.y)**2)
            theta = math.atan2((obs.y - msg.player.y), (obs.x - msg.player.x))
            if (max(self.d_obs - d, 0)) != 0.0:
                F = self.k_obs * (max(self.d_obs - d, 0))
                f_y += round(F * math.sin(theta), 2) + self.b_obs*(msg.velocity.linear.z)
                f_x += round(F * math.cos(theta), 2) + self.b_obs*(msg.velocity.linear.x)
            print "d", d
            print "velocity", msg.velocity.linear

        (position, _v, _e) = tools.helper.call_return_joint_states()

        base_force = WrenchStamped()
        base_force.header.frame_id = "base_link"
        base_force.wrench.force.x = round(f_x, 1)
        base_force.wrench.force.y = 0
        base_force.wrench.force.z = round(f_y, 1)
        self.pub_base.publish(base_force)
        f_tip = np.asarray([[0], [-round(f_x, 1)], [round(f_y, 1)]])
        #f_tip = np.asarray([[0], [0], [0]])

        return f_tip

    def goal_force(self, msg):
        pass
