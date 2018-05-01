#!/usr/bin/env python
"""
    Nathaniel Goldfarb

    This node handles the gravity assaistance
    it subscribes to other required forces and addes the nessary force to
    cancel out gravity.
"""
import rospy
from strokeRehabSystem.msg import *
from geometry_msgs.msg import Pose,Point, WrenchStamped
import tools.helper
import tools.dynamics

import controllers.GravityCompensationController as grav_controller
import numpy as np


class GravityController:

    def __init__(self):
        """
            Constructor for the gravity controller

        """
        # TODO add parameter to set K
        rospy.init_node("gravity_controller")
        rospy.Subscriber("haptic_force", udpTorque, self.gravity_callback)
        self.pub_forces = rospy.Publisher("torque_server", udpTorque, queue_size=1)
        grav_K = np.eye(3)
        grav_K[0][0] = 0.0009
        grav_K[1][1] = -0.00005
        grav_K[2][2] = -0.00005
        self.controller = grav_controller.GravityCompensationController(np.asmatrix(grav_K))

    def gravity_callback(self,force):
        """
        subscribes to a force message and add gravity to it
        :param force: udpTorque.msg
        :return:

        """
        output_force = udpTorque()
        output_force.header.frame_id = "base_link"
        (position, velocity, load) = tools.helper.call_return_joint_states()
        f_grav = self.controller.get_tau(position)
        beta = -0.001  # scale the gravity
        output_force.wrench.force.x = force.wrench.force.x + beta * f_grav[0]
        output_force.wrench.force.y = force.wrench.force.y + beta * f_grav[1]
        output_force.wrench.force.z = force.wrench.force.z + beta * f_grav[2]
        output_force.vibration = force.vibration  # grab the vibration state
        self.pub_forces.publish(output_force)


if __name__ == '__main__':
    grav = GravityController()
    rospy.spin()
