#!/usr/bin/env python

import rospy
from strokeRehabSystem.msg import *
from geometry_msgs.msg import Pose,Point, WrenchStamped
import tools.helper
import tools.dynamics

import controllers.GravityCompensationController as grav_controller
import numpy as np


class GravityController():

    def __init__(self):
        """

        """
        rospy.init_node("gravity_controller")
        rospy.Subscriber("haptic_force", WrenchStamped, self.add_gravity)
        self.pub_forces = rospy.Publisher("torque_server", WrenchStamped, queue_size=1)
        grav_K = np.eye(3)
        grav_K[0][0] = 0.0009
        grav_K[1][1] = -0.00005
        grav_K[2][2] = -0.00005
        self.controller = grav_controller.GravityCompensationController(np.asmatrix(grav_K))

    def add_gravity(self,force):
        """

        :param force:
        :return:
        """
        output_force = WrenchStamped()
        output_force.header.frame_id = "base_link"
        (position, velocity, load) = tools.helper.call_return_joint_states()
        f_grav = self.controller.get_tau(position)
        beta = -0.001
        output_force.wrench.force.x = force.wrench.force.x + beta * f_grav[0]
        output_force.wrench.force.y = force.wrench.force.y + beta * f_grav[1]
        output_force.wrench.force.z = force.wrench.force.z + beta * f_grav[2]
        self.pub_forces.publish(output_force)


if __name__ == '__main__':
    grav = GravityController()
    rospy.spin()
