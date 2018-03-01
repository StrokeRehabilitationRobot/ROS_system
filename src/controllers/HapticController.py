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
        self.pub_forces = rospy.Publisher("torque_server", WrenchStamped, queue_size=1)
        K = 800* np.identity(3)
        B = 50*np.identity(3)
        d_goal = 0.5*maze_helper.BLOCKSIZE_X + 0.5*maze_helper.PLAYERSIZE_X + 1.5*maze_helper.BLOCKSIZE_X
        d_obs = 0.5*maze_helper.BLOCKSIZE_X + 0.5*maze_helper.PLAYERSIZE_X + maze_helper.BLOCKSIZE_X
        self.odom_list = tf.TransformListener()
        self.controller = PDController.PDController(K,B)
        self.environment =  EnviromentDynamics.EnviromentDynamics(0.25,1,0.001,0.001,d_obs,d_goal)
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
        F = self.calc_arm_input()
        #add environmental pub_forces
        F_env = self.environment.make_force(haptic)
        #print F_env
        self.move(F)
        F_plane = self.calc_plane_forces()

        #output forces to arm
        output_force = WrenchStamped()
        output_force.header.frame_id = "base_link"
        [output_force.wrench.force.x, output_force.wrench.force.y, output_force.wrench.force.z] = F_env
        self.pub_forces.publish(output_force)


    def calc_arm_input(self):
        (position, velocity, _) = tools.helper.call_return_joint_states()
        self.odom_list.waitForTransform('base_link', 'master_EE', rospy.Time(0),rospy.Duration(0.1))
        (task_position, _ ) = self.odom_list.lookupTransform('base_link', 'master_EE', rospy.Time(0))
        (j1,j2,j3) = tools.dynamics.get_jacobian_matricies(position)
        task_velocity = np.array(j3).dot(np.array(velocity).reshape(3, 1))
        e =  self.state[0:3]-np.array(task_position).reshape(3, 1)
        ed =  self.state[3:]-np.array(task_velocity[0:3]).reshape(3, 1)
        F = self.controller.get_F(e,ed)
        F = np.round(F,2)
        return F

    def calc_plane_forces(self):
        (position, velocity, _) = tools.helper.call_return_joint_states()
        self.odom_list.waitForTransform('base_link', 'master_EE', rospy.Time(0),rospy.Duration(0.1))
        (task_position, _ ) = self.odom_list.lookupTransform('base_link', 'master_EE', rospy.Time(0))
        (j1,j2,j3) = tools.dynamics.get_jacobian_matricies(position)
        task_velocity = np.array(j3).dot(np.array(velocity).reshape(3, 1))
        e =  self.state[0:3]-np.array(task_position).reshape(3, 1)
        ed =  self.state[3:]-np.array(task_velocity[0:3]).reshape(3, 1)
        F = self.controller.get_F(e,ed)
        F = np.round(F,2)
        return F

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
