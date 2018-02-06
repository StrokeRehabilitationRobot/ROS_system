#!/usr/bin/env python
import pygame
from pygame.locals import *
import maze_helper
import math
import mazeBank
import Player
import numpy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose,Point
import rospy
import tf
# Colors for use throughout
RED = (255,0,0)
GREEN = (0,255,0)
BLUE = (0,0,255)
DARK_BLUE = (0,0,128)
WHITE = (255,255,255)
BLACK = (0,0,0)
PINK = (255,200,200)
PURPLE = (255,150,255)

# Map element sizes
BLOCKSIZE_X = 50
BLOCKSIZE_Y = 50
PLAYERSIZE_X = 20
PLAYERSIZE_Y = 20

# Translating arm motion to map
THRESHOLD = 0.05
Y_CUTOFF = 0.35



class Maze:
    windowWidth = 800
    windowHeight = 600
    def __init__(self, maze_name="maze1"):
        """

        :param maze_name:
        """
        rospy.init_node('MazeGame', anonymous=True)
        rospy.Subscriber("Maze", OccupancyGrid, self.maze_callback)
        rospy.Timer(rospy.Duration(0.01), self.update)
        self.pub_player = rospy.Publisher('Player', Point, queue_size=1)
        self.player = Point()
        self._odom_list = tf.TransformListener()
        self.N = len(self.maze) # number of rows
        self.M = len(self.maze[0]) # number of columns
        self.on_init()


    def on_init(self):

        pygame.init()
        self.display_surf = pygame.display.set_mode((self.windowWidth, self.windowHeight))
        pygame.display.set_caption('Travel from Blue Square to Red Square')
        self._running = True
        start = maze_helper.getStart(self.maze)
        goal = maze_helper.getGoal(self.maze)
        start_loc_pixels_x = (start[0] * BLOCKSIZE_X) + math.floor(abs((BLOCKSIZE_X - PLAYERSIZE_X) * 0.5))
        start_loc_pixels_y = (start[1] * BLOCKSIZE_Y) + math.floor(abs((BLOCKSIZE_Y - PLAYERSIZE_Y) * 0.5))
        pygame.display.update()

    def update(self):
        """

        :return:
        """
        arm_translation_x = 0.25
        arm_scale_x = 0.35
        arm_translation_y = 0.15
        arm_scale_y = 0.35


        self._odom_list.waitForTransform('base_link', 'master_EE', rospy.Time(0), rospy.Duration(1.0))
        (position, orientation) = self._odom_list.lookupTransform('base_link', 'master_EE', rospy.Time(0))

        EE_x = ((arm_translation_x - position[0]) / arm_scale_x)*self.maze.info.width
        EE_y =  ((position[1] - arm_translation_y) / arm_scale_y)*self.maze.info.height
        (self.player.x,self.player.y) = numpy.multiply([EE_x, EE_y],[BLOCKSIZE_X, BLOCKSIZE_Y])


        pygame.draw.rect(self.display_surf, GREEN,
                         (self.player.x, self.player.y, PLAYERSIZE_X, PLAYERSIZE_Y), 0)


        self.pub_player(self.player)



    def maze_callback(self,msg):
        self.maze = msg


    def invert(self):
        for index, row in enumerate(self.maze):
            self.maze[index] = row[::-1]


    def draw(self):
        # Iterate over maze
        
        for index, pt in enumerate(self.maze.data):
            bx,by = maze_helper.get_i_j(self.maze,index)

            cell = maze_helper.check_cell(self.maze,index)
            if cell == 1:
                pygame.draw.rect(self.display_surf, PURPLE,
                                 (bx * BLOCKSIZE_X, by * BLOCKSIZE_Y, BLOCKSIZE_X, BLOCKSIZE_Y), 0)
            elif cell == 2:
                pygame.draw.rect(self.display_surf, BLUE,
                                 (bx * BLOCKSIZE_X, by * BLOCKSIZE_Y, BLOCKSIZE_X, BLOCKSIZE_Y), 0)
            elif cell == 3:
                pygame.draw.rect(self.display_surf, RED,
                                 (bx * BLOCKSIZE_X, by * BLOCKSIZE_Y, BLOCKSIZE_X, BLOCKSIZE_Y), 0)


    def is_start(self):

        start = maze_helper.getStart(self.maze)
        start_pixels = (start[0] * BLOCKSIZE_X, start[1] * BLOCKSIZE_Y)

        return abs(self.player.x - start_pixels[0]) < PLAYERSIZE_X and \
               abs(self.player.y - start_pixels[1]) < PLAYERSIZE_Y


    def is_done(self):
        goal = maze_helper.getGoal(self.maze)
        goal_pixels = (goal[0] * BLOCKSIZE_X, goal[1] * BLOCKSIZE_Y)

        return abs(self.player.x - goal_pixels[0]) < PLAYERSIZE_X and \
               abs(self.player.y - goal_pixels[1]) < PLAYERSIZE_Y


if __name__ == "__main__":
    theApp = App()
    theApp.on_execute()
