#!/usr/bin/env python

import sys
import pygame
from pygame.locals import *
import maze_helper
import math
import mazeBank
import numpy as np
from nav_msgs.msg import OccupancyGrid,Path
from strokeRehabSystem.srv import ReturnJointStates
from geometry_msgs.msg import Pose,Point, WrenchStamped
from std_msgs.msg import Bool
import rospy
import time
import tf
import tools.joint_states_listener
import tools.helper
import controllers.HapticController


# Colors for use throughout
RED = (255,0,0)
PEACH = (255,100,100)
YELLOW = (200,200,0)
GREEN = (0,255,0)
MINT = (100,255,175)
AQUA = (0,150,150)
BLUE = (0,0,255)
DARK_BLUE = (0,0,128)
PINK = (255,200,200)
PURPLE = (255,150,255)
MAGENTA = (100,0,100)
WHITE = (255,255,255)
BLACK = (0,0,0)

# Map element sizes
BLOCKSIZE_X = 30
BLOCKSIZE_Y = 30
PLAYERSIZE_X = 10
PLAYERSIZE_Y = 10


class Maze:
    windowWidth = 1000
    windowHeight = 600
    def __init__(self, maze_name="maze1"):
        """

        :param maze_name:
        """

        #self.walls = []
        self.starts = []
        self.goals = []
        self.goal_rec = None
        self.start_rec = None
        # AV_TODO: can use player = player_rec?
        self.player = Point()
        (self.player.x, self.player.y) =  (self.windowWidth*0.5,self.windowHeight*0.5)
        self.player_rec = pygame.Rect((self.player.x, self.player.y, PLAYERSIZE_X, PLAYERSIZE_Y) )

        self.solved_path = Path()
        self.score = 0
        self.wall_force = [0, 0, 0]
        self.game_timer = 0

        self.running = False
        self.am_i_at_goal = False
        self.am_i_at_start = False

        rospy.init_node('MazeGame', anonymous=True)
        rospy.Subscriber("gen_maze", OccupancyGrid, self.maze_callback)
        rospy.Subscriber("a_star", Path, self.path_callback)
        rospy.Timer(rospy.Duration(0.1), self.update_GUI)
        self.pub_player = rospy.Publisher('Player', Point, queue_size=1)
        self.pub_goal   = rospy.Publisher('at_goal', Bool, queue_size=1)
        self.pub_start  = rospy.Publisher('at_start', Bool, queue_size=1)
        self.pub_forces = rospy.Publisher("motors_server", WrenchStamped, queue_size=1)

        d_goal = 0.5*BLOCKSIZE_X + 0.5*PLAYERSIZE_X + 1.5*BLOCKSIZE_X
        d_obs = 0.5*BLOCKSIZE_X + 0.5*PLAYERSIZE_X + BLOCKSIZE_X
        self.controller = controllers.HapticController.HapticController(0.01,0.001,d_obs,d_goal)
        self.controller.zero_force()

        pygame.init()
        pygame.font.init()
        self.myfont = pygame.font.SysFont('Comic Sans MS', 18)


    def on_init(self):
        """
        sets up the game
        :return:
        """
        self.display_surf = pygame.display.set_mode((self.windowWidth, self.windowHeight))
        self.running = True
        self.am_i_at_goal = False
        self.am_i_at_start = False
        self.score = 0
        self.game_timer = time.time()
        pygame.display.set_caption('Travel from Blue Square to Red Square')
        self.N = self.maze.info.height  # number of rows
        self.M = self.maze.info.width  # number of columns
        self.maze_draw()
        self.player_draw()
        pygame.display.update()
        self.controller.zero_force()

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
            if self.am_i_at_start:
                self.am_i_at_goal = self.at_goal()
            else:
                self.am_i_at_start = self.at_start()
            self.player_draw()
            scoretext = "Current Score: %d, Time Elapsed: %d s" %(self.score, time.time() - self.game_timer)
            textsurface = self.myfont.render(scoretext, False, WHITE)
            pygame.display.update()
            self.display_surf.blit(textsurface, (0,0))
            #self.pub_player.publish(self.player)


    def maze_callback(self,msg):
        """
        saves the maze and sets up the game
        :param msg: occupany grid message
        :return:
        """
        #AV_TODO: should we ditch this callback and have the subscriber just jump to on_init?
        self.maze = msg
        self.on_init()

    def invert(self):
        for index, row in enumerate(self.maze):
            self.maze[index] = row[::-1]

    def player_draw(self):
        """
        draws the player location
        :return:
        """
        # start = self.at_start()
        # goal  = self.at_goal()

        (self.player.x, self.player.y) =  tools.helper.robot_to_game((0,self.windowWidth), (0,self.windowHeight)  )
        self.player_rec = pygame.Rect((self.player.x, self.player.y, PLAYERSIZE_X, PLAYERSIZE_Y) )
        # AV_TODO: can use player_center = player_rec.center?
        player_center = Point()
        player_center.x = self.player_rec.centerx
        player_center.y = self.player_rec.centery
        wall_centers = self.check_collision_adaptive()
        goal_centers = self.goal_adaptive()
        self.update_score()
        if self.am_i_at_start:
            self.controller.make_force(player_center,wall_centers,goal_centers)

        pygame.draw.rect(self.display_surf, WHITE,
                         (self.player.x, self.player.y, PLAYERSIZE_X, PLAYERSIZE_Y), 0)


    def maze_draw(self):
        """
        callback for the maze
        draws the maze
        :return:
        """

        for index, pt in enumerate(self.maze.data):
            bx,by = maze_helper.get_i_j(self.maze,index)
            cell = maze_helper.check_cell(self.maze,index)
            if cell == 1:
                pygame.draw.rect(self.display_surf, PURPLE,
                                 (bx * BLOCKSIZE_X, by * BLOCKSIZE_Y, BLOCKSIZE_X, BLOCKSIZE_Y), 0)
                #self.walls.append(pygame.Rect(bx * BLOCKSIZE_X, by * BLOCKSIZE_Y, BLOCKSIZE_X, BLOCKSIZE_Y))

            elif cell == 2:
                pygame.draw.rect(self.display_surf, BLUE,
                                 (bx * BLOCKSIZE_X, by * BLOCKSIZE_Y, BLOCKSIZE_X, BLOCKSIZE_Y), 0)
                self.starts.append(pygame.Rect(bx * BLOCKSIZE_X, by * BLOCKSIZE_Y, BLOCKSIZE_X, BLOCKSIZE_Y))
                self.start_rec = pygame.rect.unionall(self.starts)

            elif cell == 3:
                pygame.draw.rect(self.display_surf, RED,
                                 (bx * BLOCKSIZE_X, by * BLOCKSIZE_Y, BLOCKSIZE_X, BLOCKSIZE_Y), 0)
                self.goals.append(pygame.Rect(bx * BLOCKSIZE_X, by * BLOCKSIZE_Y, BLOCKSIZE_X, BLOCKSIZE_Y))
                self.goal_rec = pygame.rect.unionall(self.goals)




    def path_callback(self,msg):
        """
        call_back for the optimal path
        sets the path
        :param msg: Path message
        :return:
        """

        self.solved_path = msg


    def path_draw(self):
        """
        draws the path
        :return:
        """
        for point in self.solved_path.poses:

            pixels_x = (point.pose.position.x * BLOCKSIZE_X) + math.floor(abs((BLOCKSIZE_X - PLAYERSIZE_X) * 0.5))
            pixels_y = (point.pose.position.y * BLOCKSIZE_Y) + math.floor(abs((BLOCKSIZE_Y - PLAYERSIZE_Y) * 0.5))
            pygame.draw.rect(self.display_surf, GREEN,
                             (pixels_x, pixels_y, PLAYERSIZE_X, PLAYERSIZE_Y), 0)


    def at_start(self):
        """
        checks if we are at the starting location
        :return: boolean check if we are at the starting location


        start = maze_helper.getStart(self.maze)
        start_pixels = (start[0] * BLOCKSIZE_X, start[1] * BLOCKSIZE_Y)
        state = Bool()
        state.data = abs(self.player.x - start_pixels[0]) < PLAYERSIZE_X and \
                     abs(self.player.y - start_pixels[1]) < PLAYERSIZE_Y
        """
        state = Bool()
        state.data = self.goal_rec.contains(self.player_rec)

        #self.pub_start.publish(state)
        return state.data

    def at_goal(self):
        """
        checks if we are at the goal
        :return: boolean check if we are at goal
        """
        #goal = maze_helper.getGoal(self.maze)
        #goal_pixels = (self.goal_rec.x, self.goal_rec.y)#(goal[0] * BLOCKSIZE_X, goal[1] * BLOCKSIZE_Y)
        state = Bool()
        state.data = self.goal_rec.contains(self.player_rec )#abs(self.player.x - goal_pixels[0]) < PLAYERSIZE_X and \
                    #abs(self.player.y - goal_pixels[1]) < PLAYERSIZE_Y
        if state.data:
            self.pub_goal.publish(state)
            self.running = False
            time_score = 20 - (time.time() - self.game_timer)
            self.score += time_score

        return state.data


    def check_collision_adaptive(self):
        #walls = []
        centers = []
        player_x = math.floor(float(self.player_rec.centerx)/BLOCKSIZE_X) # This is the (x,y) block in the grid where the top left corner of the player is
        player_y = math.floor(float(self.player_rec.centery)/BLOCKSIZE_Y)
        for x in range(int(player_x) - 1, int(player_x) + 2):
            for y in range(int(player_y) - 1, int(player_y) + 2):
                point_index = maze_helper.index_to_cell(self.maze, x, y)
                neighbors = maze_helper.neighbors_manhattan(self.maze, x,y)
                #goal = maze_helper.getGoal(self.maze)
                if maze_helper.check_cell(self.maze, int(point_index)) == 1 and not (self.goals | neighbors):
                    point = Point()
                    wall_block = pygame.Rect((x * BLOCKSIZE_X, y * BLOCKSIZE_Y, BLOCKSIZE_X, BLOCKSIZE_Y))
                    point.x = wall_block.centerx
                    point.y = wall_block.centery
                    centers.append(point)
                    #walls.append(wall_block)
                    pygame.draw.rect(self.display_surf, GREEN , wall_block, 0)

        pygame.display.update()
        return centers

    def goal_adaptive(self):

        goal = Point()
        start = Point()
        points = []
        goal.x = self.goal_rec.centerx
        goal.y = self.goal_rec.centery
        start.x = self.start_rec.centerx
        start.y = self.start_rec.centery
        points.append(start)

        if self.solved_path.poses:
            for pose in self.solved_path.poses:
                pt = Point()
                pt.x = (pose.pose.position.x * BLOCKSIZE_X) + math.floor(abs((BLOCKSIZE_X - PLAYERSIZE_X) * 0.5))
                pt.y = (pose.pose.position.y * BLOCKSIZE_Y) + math.floor(abs((BLOCKSIZE_Y - PLAYERSIZE_Y) * 0.5))
                print pt
                points.append(pt)

        points.append(goal)

        return points

    def update_score(self, assistance=3):

        player_x = math.floor(float(self.player_rec.centerx) / BLOCKSIZE_X)  # This is the (x,y) block in the grid where the center of the player is
        player_y = math.floor(float(self.player_rec.centery) / BLOCKSIZE_Y)
        point_index = maze_helper.index_to_cell(self.maze, player_x, player_y)
        if maze_helper.check_cell(self.maze, int(point_index)) == 1:
            self.score -= 1
        for pose in self.solved_path.poses:
            if pose.pose.position.x == player_x and pose.pose.position.y == player_y:
                reward = math.floor(1./len(self.solved_path) * 50)
                self.score += reward


if __name__ == "__main__":

   game = Maze()
   while not rospy.is_shutdown():
       rospy.spin()
