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
import GravityCompensationController
import games.maze.maze_helper as maze_helper
import time
from sensor_msgs.msg import JointState
from tools import dynamics

class HapticController():

    def __init__(self):
        """
        """
        rospy.init_node("haptic_controller")
        rospy.Subscriber("enviroment", hapticForce, self.make_forces)
        self.pub_player = rospy.Publisher('Player', Point, queue_size=1)
        self.pub_forces = rospy.Publisher("haptic_force", WrenchStamped, queue_size=1)
        self.pub_move_player = rospy.Publisher("move_player", JointState, queue_size=1)

        self.mass = 10


        K = 500 * np.identity(3)
        B = 50 * np.identity(3)
        d_obs = 0.02

        self.odom_list = tf.TransformListener()
        self.odom_list.waitForTransform('base_link', 'master_EE', rospy.Time(0), rospy.Duration(0.1))
        (task_position, _) = self.odom_list.lookupTransform('base_link', 'master_EE', rospy.Time(0))
        (position, velocity, load) = tools.helper.call_return_joint_states()

        x = task_position[0]
        y = task_position[1]
        z = task_position[2]

        self.player = PlayerModel.PlayerModel(self.mass,(x,y,z))
        self.player.state = np.array([[x], [y], [z], [0], [0], [0]])
        self.environment = WallForces.WallForces(500, 50, d_obs)

        self.controller = PDController.PDController(K, B)
        self.time0 = time.clock()
        self.prev_angles = np.asarray(position).reshape(3,1)

    def make_forces(self, haptic):
        """
            subscriber to the haptic force
            gets the joints states from listener
            pass into the controller and Enviroment
            computers the forces
        """
        alpha = -0.05

        (position, velocity, load) = tools.helper.call_return_joint_states()

        f_env = self.environment.make_force(self.player,haptic)
        f_arm = self.calc_arm_input(position, velocity)

        # TODO seperate out into node
        self.player.move(np.add(f_env ,f_arm),haptic.obstacles)
        #F = self.calc_output_force(position,velocity)

        #output forces to arm
        output_force = WrenchStamped()
        output_force.header.frame_id = "base_link"
        output_force.wrench.force.x = alpha * ( f_env[2] )
        output_force.wrench.force.y = alpha * ( f_env[0] )
        output_force.wrench.force.z = alpha * ( f_env[0] )


        self.pub_forces.publish(output_force)


    def calc_arm_input(self,position,velocity):
        """
        calculates the input force from the arm
        :return: arm force
        """
        self.odom_list.waitForTransform('base_link', 'master_EE', rospy.Time(0), rospy.Duration(0.1))
        (task_position, _) = self.odom_list.lookupTransform('base_link', 'master_EE', rospy.Time(0))
        (j1,j2,j3) = tools.dynamics.get_jacobian_matricies(position)
        j3 = j3[0:3,0:3]
        task_velocity = np.array(j3).dot(np.array(velocity).reshape(3, 1))
        e  = self.player.state[0:3]-np.array(task_position).reshape(3, 1)
        ed = self.player.state[3:]-np.array(task_velocity[0:3]).reshape(3, 1)
        F  = 100*self.controller.get_F(e,ed)
        F  = np.round(F,2)

        return F


    def goal_force(self,goal):
        pass


if __name__ == '__main__':
    haptic = HapticController()
    rospy.spin()
