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

            d = math.sqrt((obs.x - msg.player.x)**2 +
                          (obs.y - msg.player.y)**2)
            theta = math.atan2((obs.y - msg.player.y), (obs.x - msg.player.x))
            F = self.k_obs * (max(self.d_obs - d, 0))
            f_y += round(F * math.sin(theta), 2)  # - self.b_obs*msg.v[1]
            f_x += round(F * math.cos(theta), 2)  # - self.b_obs*msg.v[0]

        (position, _v, _e) = tools.helper.call_return_joint_states()
        rot_mat = self.rotation_matrix()

        if abs(f_x) < 0.25:
            f_x = 0
        if abs(f_y) < 0.25:
            f_y = 0

        #print "Base frame motions: x(left): %.2f, z(up): %.2f" %(-round(f_x,1), -round(f_y, 1))
        base_force = WrenchStamped()
        base_force.header.frame_id = "base_link"
        base_force.wrench.force.x = 0
        base_force.wrench.force.y = round(f_x, 1)
        base_force.wrench.force.z = -round(f_y, 1)
        self.pub_base.publish(base_force)
        f_tip = np.dot(np.array(rot_mat), [0, round(f_x, 1), -round(f_y, 1)]).reshape(3, 1)
        #f_tip = np.dot(np.array(rot_mat), [0, 1, 0]).reshape(3, 1)
        tip_force = WrenchStamped()
        tip_force.header.frame_id = "master_EE"
        tip_force.wrench.force.x = f_tip[0]
        tip_force.wrench.force.y = f_tip[1]
        tip_force.wrench.force.z = f_tip[2]
        self.pub_tip.publish(tip_force)

        #print "(tip_x, tip_y, tip_z) = (%.1f, %.1f, %.1f)" %(f_tip[0], f_tip[1], f_tip[2])

        f_tip = [f_tip[0], f_tip[1], f_tip[2]]
        return f_tip

    def rotation_matrix(self):

        (position, velocity, _) = tools.helper.call_return_joint_states()
        self.odom_list.waitForTransform('master_EE', 'base_link', rospy.Time(0), rospy.Duration(0.1))
        (task_position, rotation_matrix) = self.odom_list.lookupTransform('master_EE', 'base_link', rospy.Time(0))
        rotation_matrix = self.odom_list.fromTranslationRotation(task_position, rotation_matrix)
        # rotation_matrix = np.matrix([[    cos(position[2] + 1.57)*cos(position[0])*cos(position[1]) - sin(position[2] + 1.57)*cos(position[0])*sin(position[1]),   - cos(position[2] + 1.57)*cos(position[0])*sin(position[1]) - sin(position[2] + 1.57)*cos(position[0])*cos(position[1]),  sin(position[0])],
        #                              [ sin(position[2] + 1.57)*( - sin(position[0])*sin(position[1])) + cos(position[2] + 1.57)*( cos(position[1])*sin(position[0])), cos(position[2] + 1.57)*( - sin(position[0])*sin(position[1])) - sin(position[2] + 1.57)*(cos(position[1])*sin(position[0])),      -cos(position[0])],
        #                              [                                      cos(position[2] + 1.57)*sin(position[1]) + sin(position[2] + 1.57)*cos(position[1]),                                       cos(position[2] + 1.57)*cos(position[1]) - sin(position[2] + 1.57)*sin(position[1]),                 0]])
        #

        # rotation_matrix = np.matrix([
        #                              [cos(position[2] + 1.57) * cos(position[0] + 1.57) * sin(position[1]) - sin(position[2] + 1.57) * cos(position[0] + 1.57) * cos(position[1]), -sin(position[0] + 1.57), cos(position[2] + 1.57) * cos(position[0] + 1.57) * cos(position[1]) - sin(position[2] + 1.57) * cos(position[0] + 1.57) * sin(position[1])],
        #                              [-0.707 * cos(position[2] + 1.57) * (cos(position[1]) + sin(position[0] + 1.57) * sin(position[1])) - sin(position[2] + 1.57) * (sin(position[1]) * (- 0.707) + 0.707 * sin(position[0] + 1.57) * cos(position[1])), 0.707 * cos(position[0] + 1.57) + 0.707 * sin(position[0] + 1.57) * cos(position[1]) - 0.707 * sin(position[0] + 1.57) * sin(position[1]), cos(position[2] + 1.57) * (sin(position[1]) * (- 0.707) + 0.707 * sin(position[0] + 1.57) * cos(position[1])) + sin(position[2] + 1.57) * (cos(position[1]) * (- 0.707) - 0.707 * sin(position[0] + 1.57) * sin(position[1]))],
        #                              [sin(position[2] + 1.57) * (sin(position[1]) * (0.707) + 0.707 * sin(position[0] + 1.57) * cos(position[1])) - cos(position[2] + 1.57) * (cos(position[1]) * (0.707) - 0.707 * sin(position[0] + 1.57) * sin(position[1])), 0.707 * sin(position[0] + 1.57) * cos(position[1]) - 0.707 * cos(position[0] + 1.57) - 0.707 * sin(position[0] + 1.57) * sin(position[1]), -cos(position[2] + 1.57) * (sin(position[1]) * (0.707) + 0.707 * sin(position[0] + 1.57) * cos(position[1])) - sin(position[2] + 1.57) * (cos(position[1]) * (0.707) - 0.707 * sin(position[0] + 1.57) * sin(position[1]))]
        #                              ])

        # q0 = position[0]
        # q1 = position[1]#-math.pi*0.25
        # q2 = position[2]
        # rotation_matrix = np.matrix([
        #                             [ -cos(q2 + 1.57)*cos(q0 + 1.57)*sin(q1) - sin(q2 + 1.57)*cos(q0 + 1.57)*cos(q1),\
        #
        #                             - sin(q0 + 1.57),\
        #
        #                             cos(q2 + 1.57)*cos(q0 + 1.57)*cos(q1) - sin(q2 + 1.57)*cos(q0 + 1.57)*sin(q1)],\
        #
        #                             [ -0.707*cos(q2 + 1.57)*(cos(q1) + sin(q0 + 1.57)*sin(q1)) + 0.707*sin(q2 + 1.57)*(sin(q1) - sin(q0 + 1.57)*cos(q1)),\
        #
        #                             0.707*cos(q0 + 1.57), \
        #
        #                             -0.707*cos(q2 + 1.57)*(sin(q1) - sin(q0 + 1.57)*cos(q1)) + -0.707*sin(q2 + 1.57)*(cos(q1) + sin(q0 + 1.57)*sin(q1))],\
        #
        #                             [0.707*sin(q2 + 1.57)*(sin(q1) + sin(q0 + 1.57)*cos(q1)) - 0.707*cos(q2 + 1.57)*(cos(q1) - sin(q0 + 1.57)*sin(q1)),\
        #
        #                             0.707*sin(q0 + 1.57)*cos(q1) - 0.707*cos(q0 + 1.57),\
        #
        #                             -0.707*cos(q2 + 1.57)*(sin(q1) + sin(q0 + 1.57)*cos(q1)) - 0.707*sin(q2 + 1.57)*(cos(q1) - sin(q0 + 1.57)*sin(q1))] \
        #
        #                             ])
        #
        # rotation_matrix=np.transpose(rotation_matrix)
        rotation_matrix = rotation_matrix[0:3,0:3]
        #print rotation_matrix
        return rotation_matrix
