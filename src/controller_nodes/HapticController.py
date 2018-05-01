#!/usr/bin/env python
"""
    This node handles the enviromental forces includeing the wall and guiding
    NOTE: This only works for the master arm. To make work for other arms the controller has to be set
    to listen to the other frames

"""

import sys
import rospy
from strokeRehabSystem.msg import *
from nav_msgs.msg import Path
import models.PlayerModel
import tf
import numpy as np
import controllers.PDController
import controllers.WallForces
import controllers.DMPController as DMP
import tools.helper
import tools.dynamics
import games.maze.maze_helper as maze_helper
import time


class HapticController():
    def __init__(self):
        """
        Set up the nessary class varibles
        """
        rospy.init_node("haptic_controller")
        # player mass, PD values
        mass = 10
        k = 500 * np.identity(3)
        b = 50 * np.identity(3)
        d_obs = 0.02  # obstical react distance
        # varibles for guiding force
        self.useing_guide = False
        self.goals = []
        self.goal_index = 0

        # frame listerners
        self.odom_list = tf.TransformListener()
        self.odom_list.waitForTransform('base_link', 'master_EE', rospy.Time(0), rospy.Duration(0.1))
        (task_position, _) = self.odom_list.lookupTransform('base_link', 'master_EE', rospy.Time(0))

        x = task_position[0]
        y = task_position[1]
        z = task_position[2]

        # controller set up
        self.player = models.PlayerModel.PlayerModel(mass, (x, y, z))
        self.player.state = np.array([[x], [y], [z], [0], [0], [0]])
        self.environment = controllers.WallForces.WallForces(500, 50, d_obs)
        self.controller = controllers.PDController.PDController(k, b)
        self.goal_runner = DMP.DMPController()

        # pubs/subs
        rospy.Subscriber("enviroment", hapticForce, self.make_forces_callback)
        self.pub_forces = rospy.Publisher("haptic_force", udpTorque, queue_size=1)
        rospy.Subscriber("a_star", Path, self.path_callback)

    def make_forces_callback(self, haptic):

        """
            subscriber to the haptic force topic and create the envirometal force.
            It then move the player in the game and applies the apporiote force the the robot.

            :param haptic: enviroment state
            :type haptic: haptic.msg
        """

        (position, velocity, load) = tools.helper.call_return_joint_states()

        f_wall = self.environment.make_force(self.player, haptic)  # calcualate the wall forces
        vib = int(abs(np.sum(f_wall)) > 0)  # if we hit a wall then vibrate the handle
        f_arm = self.calc_arm_input(position, velocity)  # PD controller on the arm input
        f_goal = np.array([[0], [0], [0]])

        # check if we are using the goal forces
        if self.useing_guide:
            f_goal = self.calc_dmp(f_wall)
            f_ext = np.add(f_wall, f_goal)
            f_total = np.add(f_ext, f_arm)
            self.player.move(f_total, haptic.obstacles)
        else:
            self.player.move(np.add(f_wall, f_arm), haptic.obstacles)

        # output forces to arm
        alpha_wall = -0.005  # scaler
        alpha_goal = 0.000009  # scaler
        output_force = udpTorque()
        output_force.header.frame_id = "base_link"
        output_force.wrench.force.x = alpha_wall * f_wall[2] + alpha_goal * f_goal[2]
        output_force.wrench.force.y = alpha_wall * f_wall[0] + alpha_goal * f_goal[0]
        output_force.wrench.force.z = alpha_wall * f_wall[1] + alpha_goal * f_goal[1]
        output_force.vibration = vib
        self.pub_forces.publish(output_force)

    def path_callback(self, msg):
        """
        callback for A* guiding force
        It waits for a path to be published and then sets  up the DMP
        :param msg: A* path
        :type msg: Path.msg
        :return: None
        """
        self.useing_guide = True
        min_dist = 1000000000000000000
        index = 0
        min_index = 0

        # find the closested point to use as the starting
        # TODO ensure the starting point is not behind the player
        for point in msg.poses:
            # get the point in the correct space
            # TODO this is coupled with the game right now, find decoupleing method -> ROS parameters???
            (px, py) = maze_helper.game_to_task(30 * point.pose.position.x + 10, 30 * point.pose.position.y + 10)

            dist = tools.helper.distance((self.player.state[1], self.player.state[2]), (px, py))
            if dist < min_dist:
                min_dist = dist
                min_index = index

            index += min_index
            self.goals.append((px, py))

        self.goal_index = min_index
        dmp_file = DMP.dmp_chooser((self.player.state[1], self.player.state[2]), self.goals[self.goal_index])
        # TODO remove abs dir
        full_path = '/home/cibr-strokerehab/CIBR_ws/src/strokeRehabSystem/xml_motion_data/'
        goal = list(self.goals[self.goal_index])
        goal.append(np.asscalar(self.player.state[0]))
        self.goal_runner.update_dmp_file(full_path + dmp_file,
                                         (self.player.state[1], self.player.state[2], self.player.state[0]), goal)

    def calc_arm_input(self, position, velocity):
        """
        calculates the input force from the arm
        :return: arm force
        :param position: EE position
        :type position: array
        :param velocity: EE velocity
        :type velocity: array
        :return: np.array(3,1)
        """

        self.odom_list.waitForTransform('base_link', 'master_EE', rospy.Time(0), rospy.Duration(0.1))
        (task_position, _) = self.odom_list.lookupTransform('base_link', 'master_EE', rospy.Time(0))
        (j1, j2, j3) = tools.dynamics.get_jacobian_matricies(position)
        j3 = j3[0:3, 0:3]
        task_velocity = np.array(j3).dot(np.array(velocity).reshape(3, 1))
        e = self.player.state[0:3] - np.array(task_position).reshape(3, 1)
        ed = self.player.state[3:] - np.array(task_velocity[0:3]).reshape(3, 1)
        f = 100 * self.controller.get_force(e, ed)
        f = np.round(f, 2)

        return f

    def calc_dmp(self, f_env):
        """
        calcualte the dmp force to apply
        :param f_env: the enviromental force
        :type f_env: np.array(3,1)
        :return: np.array(3,1) of the force vector
        """

        (x, y) = (self.goals[self.goal_index][0], self.goals[self.goal_index][1])
        dist = tools.helper.distance((self.player.state[1], self.player.state[2]), (x, y))
        # TODO remove abs path
        full_path = '/home/cibr-strokerehab/CIBR_ws/src/strokeRehabSystem/xml_motion_data/'

        # if close to the DMP and there are still more waypoints get the next waypoint
        if dist < 0.02 and self.goal_index < len(self.goals) - 1:
            self.goal_index += 1
            self.player.stop()
            dmp_file = DMP.dmp_chooser((self.player.state[0], self.player.state[2]), self.goals[self.goal_index])
            goal = list(self.goals[self.goal_index])
            goal.append(np.asscalar(self.player.state[0]))
            self.goal_runner.update_dmp_file(full_path + dmp_file,
                                             (self.player.state[1], self.player.state[2], self.player.state[0]),
                                             goal)

        dt = 0.001
        tau = 0.8
        f = self.goal_runner.step(tau, dt, self.player.state, f_env)
        return 0.1 * f


if __name__ == '__main__':
    haptic = HapticController()
    rospy.spin()
