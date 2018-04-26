#!/usr/bin/env python

import sys
import rospy
from strokeRehabSystem.msg import *
from geometry_msgs.msg import Pose,Point, WrenchStamped
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
from sensor_msgs.msg import JointState


class HapticController():

    def __init__(self):
        """
        """
        rospy.init_node("haptic_controller")


        self.mass = 10
        K = 500 * np.identity(3)
        B = 50 * np.identity(3)
        d_obs = 0.02
        self.count = 0
        self.odom_list = tf.TransformListener()
        self.odom_list.waitForTransform('base_link', 'master_EE', rospy.Time(0), rospy.Duration(0.1))
        (task_position, _) = self.odom_list.lookupTransform('base_link', 'master_EE', rospy.Time(0))
        (position, velocity, load) = tools.helper.call_return_joint_states()

        self.useing_guide = False
        self.goals = []
        self.goal_index  = 0
        self.goal_runner = DMP.DMPController()


        x = task_position[0]
        y = task_position[1]
        z = task_position[2]

        self.player = models.PlayerModel.PlayerModel(self.mass, (x, y, z))
        self.player.state = np.array([[x], [y], [z], [0], [0], [0]])
        self.environment = controllers.WallForces.WallForces(500, 50, d_obs)

        self.controller = controllers.PDController.PDController(K, B)
        self.time0 = time.clock()
        self.prev_angles = np.asarray(position).reshape(3,1)

        rospy.Subscriber("enviroment", hapticForce, self.make_forces_callback)
        self.pub_forces = rospy.Publisher("haptic_force", WrenchStamped, queue_size=1)
        rospy.Subscriber("a_star", Path, self.path_callback)

    def make_forces_callback(self, haptic):
        """
            subscriber to the haptic force
            gets the joints states from listener
            pass into the controller and Enviroment
            computers the forces
        """

        (position, velocity, load) = tools.helper.call_return_joint_states()

        f_wall = self.environment.make_force(self.player,haptic)
        f_arm = self.calc_arm_input(position, velocity)
        f_goal = np.array([[0],[0],[0]])
        if self.useing_guide:
            f_goal = self.calc_dmp(f_wall)
            f_ext = np.add(0,f_goal)
            f_total = np.add(f_ext,0)
            self.player.move(f_total,haptic.obstacles)
        else:
            self.player.move(np.add(f_wall ,f_arm),haptic.obstacles)
        #F = self.calc_output_force(position,velocity)

        #output forces to arm
        alpha_wall = -0.005
        alpha_goal = -0.05
        output_force = WrenchStamped()
        output_force.header.frame_id = "base_link"
        output_force.wrench.force.x = alpha_wall * f_wall[2] + alpha_goal * f_goal[2]
        output_force.wrench.force.y = alpha_wall * f_wall[0] + alpha_goal * f_goal[0]
        output_force.wrench.force.z = alpha_wall * f_wall[1] + alpha_goal * f_goal[1]
        self.pub_forces.publish(output_force)


    def path_callback(self,msg):
        """

        :param path:
        :return:
        """
        self.useing_guide = True
        min_dist = 1000000000000000000
        min_index = 0
        index = 0
        for point in msg.poses:

            (px,py) = maze_helper.game_to_task(30*point.pose.position.x+10,30*point.pose.position.y+10)

            dist  = tools.helper.distance((self.player.state[1],self.player.state[2]),(px,py))
            if dist < min_dist:
                min_dist = dist
                min_index = index
            index += 1
            self.goals.append((px,py))

        self.goal_index = 1
        #print len(self.goals)
        (x,y) =  maze_helper.task_to_game(*self.goals[self.goal_index ])
        dmp_file = DMP.dmp_chooser((self.player.state[1],self.player.state[2]),self.goals[self.goal_index ])
        print dmp_file
        full_path = '/home/cibr-strokerehab/CIBR_ws/src/strokeRehabSystem/xml_motion_data/'
        #print self.goals[self.goal_index]
        #print  list( self.goals[self.goal_index])
        goal = list(self.goals[self.goal_index])#.append(self.player.state[0])
        goal.append( np.asscalar(self.player.state[0]) )

        self.goal_runner.update_dmp_file(full_path + dmp_file, (self.player.state[1],self.player.state[2],self.player.state[0]),goal )


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


    def calc_dmp(self,f_env):

        self.time0 = time.time()
        (x,y) = (self.goals[self.goal_index][0],self.goals[self.goal_index][1])
        #print "player", (self.player.state[0],self.player.state[1],self.player.state[2])
        dist = tools.helper.distance((self.player.state[1], self.player.state[2]),(x,y) )
        #print "goal", self.goals[self.goal_index ]
        full_path = '/home/cibr-strokerehab/CIBR_ws/src/strokeRehabSystem/xml_motion_data/'
        if dist < 0.005:

            self.goal_index += 1
            print self.goal_index
            self.count = 0
            self.player.stop()
            dmp_file = DMP.dmp_chooser((self.player.state[0], self.player.state[2]), self.goals[self.goal_index])
            goal = list(self.goals[self.goal_index])  # .append(self.player.state[0])
            goal.append(np.asscalar(self.player.state[0]))
            self.goal_runner.update_dmp_file(full_path + dmp_file,
                                             (self.player.state[1], self.player.state[2], self.player.state[0]),
                                             goal)

        dt = 0.001#time.time() - self.time0
        tau = 0.8
        #if self.count < (tau / dt) + 1:
        F = self.goal_runner.step(tau,dt,self.player.state,f_env)

        self.time0 = time.time()
        #time.sleep(0.1)
        return 0.05*F

if __name__ == '__main__':
    haptic = HapticController()
    rospy.spin()

















