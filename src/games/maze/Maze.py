#!/usr/bin/env python

import sys
import pygame
from pygame.locals import *
import maze_helper
import math
import mazeBank
import threading
from sensor_msgs.msg import JointState
import numpy as np
from nav_msgs.msg import OccupancyGrid,Path
from strokeRehabSystem.srv import ReturnJointStates
from strokeRehabSystem.msg import hapticForce
from geometry_msgs.msg import Pose,Point, WrenchStamped
from std_msgs.msg import Bool
import rospy
import time
import tf
import tools.joint_states_listener
import tools.helper
import tools.dynamics
import controllers.HapticController
import EnviromentDynamics
from operator import sub
import time


class Maze:



    def __init__(self, maze_name="maze1"):
        """

        :param maze_name:
        """

        self.walls = []
        self.starts = []
        self.goals = []
        self.goal_rec = None
        self.start_rec = None
        self.x_avg = []
        self.y_avg = []
        self.player = Rect((maze_helper.windowWidth*0.5, maze_helper.windowHeight*0.5, maze_helper.PLAYERSIZE_X, maze_helper.PLAYERSIZE_Y) )

        self.solved_path = []
        self.score = 0
        self.wall_force = [0, 0, 0]
        self.game_timer = 0

        self.running = False
        self.am_i_at_goal = False
        self.am_i_at_start = False
        self.time0 = 0
        self.pose_old = (0,0)
        self.csv = open("/home/cibr-strokerehab/Documents/JointStatesRecording.csv", "w")

        rospy.init_node('MazeGame', anonymous=True)
        rospy.Subscriber("gen_maze", OccupancyGrid, self.maze_callback)
        rospy.Subscriber("a_star", Path, self.path_callback)
        #rospy.Subscriber('joint_states', JointState, self.update_player)
        # rospy.Timer(rospy.Duration(0.01), self.update_player)
        # rospy.Timer(rospy.Duration(0.01), self.update_force)
        self.odom_list = tf.TransformListener()
        self.pub_player = rospy.Publisher('Player', Point, queue_size=1)
        self.pub_goal   = rospy.Publisher('at_goal', Bool, queue_size=1)
        self.pub_forces = rospy.Publisher("torque_server", WrenchStamped, queue_size=1)
        self.pub_enviroment = rospy.Publisher("haptic", hapticForce, queue_size=1)


        player_thread = threading.Thread(target=self.update_player)
        player_thread.daemon = True
        player_thread.start()
        #threading.Thread(target=self.update_GUI).start()
        force_thread = threading.Thread(target=self.update_force)
        force_thread.daemon = True
        force_thread.start()
        rospy.Timer(rospy.Duration(0.1), self.update_GUI)

        pygame.init()
        pygame.font.init()

        # self.myfont = pygame.font.SysFont('Comic Sans MS', 18)


    def on_init(self):
        """
        sets up the game
        :return:
        """
        self.display_surf = pygame.display.set_mode((maze_helper.windowWidth, maze_helper.windowHeight))

        self.running = True
        self.am_i_at_goal = False
        self.am_i_at_start = False
        self.score = 0
        self.starts = []
        self.goals = []

        self.game_timer = time.time()
        pygame.display.set_caption('Travel from Blue Square to Red Square')
        self.maze_draw()
        self.player_draw()
        pygame.display.update()
        self.pose_old = self.player


    """
        Updaters, called on timmer callbacks
    """

    def update_player(self):

        while 1:
            (x, y) =  maze_helper.task_to_game( (0,maze_helper.windowWidth), (0,maze_helper.windowHeight) )
            self.player = pygame.Rect((x, y, maze_helper.PLAYERSIZE_X, maze_helper.PLAYERSIZE_Y) )
            v = self.get_velocity()
            time.sleep(0.01)


    def update_force(self):

        while 1:
            msg = hapticForce()
            if self.running:

                player_center = Point()
                player_center.x = self.player.centerx
                player_center.y = self.player.centery
                centers, walls = maze_helper.check_collision_adaptive(self.player,self.maze)

                for wall_block in walls:
                    pygame.draw.rect(self.display_surf, maze_helper.GREEN , wall_block, 0)
                msg.player = player_center
                msg.obstacles = centers
                msg.goals = []
                self.pub_enviroment.publish(msg)
                #goal_centers = maze_helper.goal_adaptive(self.start_rec,self.goal_rec,self.path_draw)
                pygame.display.update()
            time.sleep(0.01)

    def update_GUI(self,msg):
        """
        refeshs the game on a timer callback
        :msg: not used
        :return:
        """

        if self.running:
            #AVQuestion could we speed this up by only drawing the blocks around the player's position?
            self.display_surf.fill((0, 0, 0))
            self.maze_draw()
            self.path_draw()
            self.player_draw()
            if self.am_i_at_start:
                self.am_i_at_goal = self.at_goal()
                elapsed_time = time.time() - self.game_timer
            else:
                self.am_i_at_start = self.at_start()
                elapsed_time = 0


            # scoretext = "Current Score: %d, Time Elapsed: %d s" %(self.score, elapsed_time)
            # textsurface = self.myfont.render(scoretext, False, maze_helper.WHITE)
            # self.display_surf.blit(textsurface, (0,0))
            pygame.display.update()
            #time.sleep(0.1)


    """
        callbacks (non-timers), mainly set up things
    """
    def maze_callback(self,msg):
        """
        saves the maze and sets up the game
        :param msg: occupany grid message
        :return:
        """
        self.maze = msg
        self.walls = []
        for index, pt in enumerate(self.maze.data):
            bx,by = maze_helper.get_i_j(self.maze,index)
            cell = maze_helper.check_cell(self.maze,index)
            if cell == 1:
                self.walls.append(pygame.Rect(bx * maze_helper.BLOCKSIZE_X, by * maze_helper.BLOCKSIZE_Y, maze_helper.BLOCKSIZE_X, maze_helper.BLOCKSIZE_Y))
            elif cell == 2:
                self.starts.append(pygame.Rect(bx * maze_helper.BLOCKSIZE_X, by * maze_helper.BLOCKSIZE_Y, maze_helper.BLOCKSIZE_X, maze_helper.BLOCKSIZE_Y))
                self.start_rec = self.starts[0].unionall(self.starts)
            elif cell == 3:
                self.goals.append(pygame.Rect(bx * maze_helper.BLOCKSIZE_X, by * maze_helper.BLOCKSIZE_Y, maze_helper.BLOCKSIZE_X, maze_helper.BLOCKSIZE_Y))
                self.goal_rec = self.goals[0].unionall(self.goals)

        #self.walls = np.asarray(self.walls)
        self.on_init()


    def path_callback(self,msg):
        """
        call_back for the optimal path
        sets the path
        :param msg: Path message
        :return:
        """

        path = []

        for point in msg.poses:

            pixels_x = (point.pose.position.x * maze_helper.BLOCKSIZE_X) + math.floor(abs((maze_helper.BLOCKSIZE_X - maze_helper.PLAYERSIZE_X) * 0.5))
            pixels_y = (point.pose.position.y * maze_helper.BLOCKSIZE_Y) + math.floor(abs((maze_helper.BLOCKSIZE_Y - maze_helper.PLAYERSIZE_Y) * 0.5))
            path.append(pygame.Rect(pixels_x, pixels_y, maze_helper.PLAYERSIZE_X, maze_helper.PLAYERSIZE_Y))

        self.solved_path = path

# Drawing
    def path_draw(self):
        """
        draws the path
        :return:
        """
        for rect in self.solved_path:
            pygame.draw.rect(self.display_surf, maze_helper.GREEN,rect, 0)

    def player_draw(self):
        """
        draws the player location
        :return:
        """

        if self.am_i_at_start:
            self.update_score()
            print "Score:", self.score

        pygame.draw.rect(self.display_surf, maze_helper.WHITE, self.player, 0)

    def maze_draw(self):
        """
        callback for the maze
        draws the maze
        :return:
        """

        for wall in self.walls:
                pygame.draw.rect(self.display_surf, maze_helper.PURPLE, wall, 0)
        pygame.draw.rect(self.display_surf, maze_helper.RED, self.goal_rec, 0)
        pygame.draw.rect(self.display_surf, maze_helper.BLUE, self.start_rec, 0)

    def get_velocity(self):
        dt = 0.01#(time.time() - self.time0)
        xd =  self.player.centerx - self.pose_old[0]
        yd =  self.player.centery - self.pose_old[1]
        v = (xd,yd)
        self.pose_old = (self.player.centerx, self.player.centery)
        v = (v[0]/dt,v[1]/dt)
        self.time0 = time.time()
        print "gane",v
        return v

    def at_start(self):
        """
        checks if we are at the starting location
        :return: boolean check if we are at the starting location

        """
        state = Bool()
        state.data = self.start_rec.contains(self.player)
        if state.data:
            self.game_timer = time.time()
        return state.data

    def at_goal(self):
        """
        checks if we are at the goal
        :return: boolean check if we are at goal
        """

        state = Bool()
        state.data = self.goal_rec.contains(self.player)
        if state.data:
            self.pub_goal.publish(state)
            self.running = False
            time_score = 20 - (time.time() - self.game_timer)
            self.score += time_score

        return state.data

    #
    def update_score(self, assistance=3): pass
    #
    #     player_x = math.floor(float(self.player.centerx) / maze_helper.BLOCKSIZE_X)  # This is the (x,y) block in the grid where the center of the player is
    #     player_y = math.floor(float(self.player.centery) / maze_helper.BLOCKSIZE_Y)
    #     point_index = maze_helper.index_to_cell(self.maze, player_x, player_y)
    #     if maze_helper.check_cell(self.maze, int(point_index)) == 1:
    #         self.score -= 1
    #     for rec in self.solved_path:
    #         if rec.centerx == player_x and rec.centery == player_y:
    #             #print "On track"
    #             reward = math.floor(1./len(self.solved_path) * 100)
    #             self.score += reward
    #         # else:
    #         #     print "Player (x,y):", player_x, player_y
    #         #     print "Waypoint (x,y):", pose.pose.position.x, pose.pose.position.y


if __name__ == "__main__":

    game = Maze()
    while not rospy.is_shutdown():
        rospy.spin()
