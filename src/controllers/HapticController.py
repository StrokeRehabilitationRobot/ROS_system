#!/usr/bin/env python

import sys
import rospy
from strokeRehabSystem.msg import *
from geometry_msgs.msg import Pose,Point, WrenchStamped
import math
import PlayerModel
import tf
import numpy as np
import PDController
import WallForces
import tools.helper
import tools.dynamics
import games.maze.maze_helper as maze_helper
import time

class HapticController():

    def __init__(self):
        """
        """
        rospy.init_node("models")
        rospy.Subscriber("haptic", hapticForce, self.make_forces)
        self.pub_player = rospy.Publisher('Player', Point, queue_size=1)
        self.pub_forces = rospy.Publisher("torque_server", WrenchStamped, queue_size=1)
        self.pub_move_player = rospy.Publisher("move_player", WrenchStamped, queue_size=1)

        self.mass = 10
        K = 1850* np.identity(3)
        B = 500*np.identity(3)
        d_goal = 0.5*maze_helper.BLOCKSIZE_X + 0.50*maze_helper.PLAYERSIZE_X + 1.50*maze_helper.BLOCKSIZE_X
        d_obs  = 0.5*maze_helper.BLOCKSIZE_X + 0.50*maze_helper.PLAYERSIZE_X + 0.25*maze_helper.BLOCKSIZE_X

        self.odom_list = tf.TransformListener()
        self.player = PlayerModel.PlayerModel(self.mass)

        self.odom_list.waitForTransform('base_link', 'master_EE', rospy.Time(0), rospy.Duration(0.1))
        (task_position, _) = self.odom_list.lookupTransform('base_link', 'master_EE', rospy.Time(0))
        self.player.state = np.array([[task_position[0]], [task_position[1]], [task_position[2]], [0], [0], [0]])
        self.environment =  WallForces.WallForces(10, 10, d_obs)
        self.controller = PDController.PDController(K,B)
        self.time0 = time.clock()


    def make_forces(self, haptic):
        """
        subscriber to the haptic force
        gets the joints states from listener
        pass into the controller and Enviroment
        computers the forces
        """

        F = self.calc_arm_input()
        F_env = self.environment.make_force(self.player,haptic)

        self.player.move(np.add(0, F))
        #self.move(np.add(F_env, F))

        #output forces to arm
        output_force = WrenchStamped()
        output_force.header.frame_id = "base_link"
        [output_force.wrench.force.y, output_force.wrench.force.x, output_force.wrench.force.z] = 0.005*F_env
        self.pub_forces.publish(output_force)

    def calc_arm_input(self):
        """
        calculates the input force from the arm
        :return: arm force
        """

        (position, velocity, _) = tools.helper.call_return_joint_states()
        self.odom_list.waitForTransform('base_link', 'master_EE', rospy.Time(0),rospy.Duration(0.1))
        (task_position, _ ) = self.odom_list.lookupTransform('base_link', 'master_EE', rospy.Time(0))
        (j1,j2,j3) = tools.dynamics.get_jacobian_matricies(position)
        task_velocity = -np.array(j3).dot(np.array(velocity).reshape(3, 1))
        e  = self.player.state[0:3]-np.array(task_position).reshape(3, 1)
        ed = self.player.state[3:]-np.array(task_velocity[0:3]).reshape(3, 1)
        F  = self.controller.get_F(e,ed)
        F  = np.round(F,2)

        return F

if __name__ == '__main__':
    haptic = HapticController()
    rospy.spin()
