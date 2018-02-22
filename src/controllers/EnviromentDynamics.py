#!/usr/bin/env python
import sys
import rospy
from strokeRehabSystem.msg import *
from geometry_msgs.msg import Pose,Point, WrenchStamped
import math
import games.maze.maze_helper as maze_helper

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


        f_y = 0
        f_x = 0

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
            F = self.k_obs *0.1* ( max(self.d_obs - d,0))
            f_y += round(F*math.sin(theta),2) #- self.b_obs*msg.v[1]
            f_x += round(F*math.cos(theta),2) #- self.b_obs*msg.v[0]

        if f_x > 0.0:
            print "%.2f >" %f_x
        if f_x < 0.0:
            print "%.2f <" %-f_x
        if f_y > 0.0:
            print "%.2f ^" %-f_y
        if f_y < 0.0:
            print "%.2f v" %f_y

        return  [-round(f_x,1), 0, -round(f_y, 1) ]

    def zero_force(self):
        forces = WrenchStamped()
        forces.header.frame_id = "master"
        [forces.wrench.force.x, forces.wrench.force.y, forces.wrench.force.z] = [ 0, 0, 0]
        self.pub.publish(forces)

# if __name__ == "__main__":
#     d_goal = 0.5*maze_helper.BLOCKSIZE_X + 0.5*maze_helper.PLAYERSIZE_X + 1.5*maze_helper.BLOCKSIZE_X
#     d_obs = 0.5*maze_helper.BLOCKSIZE_X + 0.5*maze_helper.PLAYERSIZE_X + maze_helper.BLOCKSIZE_X
#     controller = EnviromentDynamics(0.01,0.001,0.0001,0.0001,d_obs,d_goal)
#     while not rospy.is_shutdown():
#         rospy.spin()
