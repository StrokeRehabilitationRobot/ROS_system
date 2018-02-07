#!/usr/bin/env python
import pygame
from pygame.locals import *
import maze_helper
import math
import mazeBank
import Player
import numpy
from nav_msgs.msg import OccupancyGrid,Path
from geometry_msgs.msg import Pose,Point
from std_msgs.msg import Bool
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
        rospy.Subscriber("gen_maze", OccupancyGrid, self.maze_callback)
        rospy.Subscriber("a_star", Path, self.maze_callback)
        rospy.Timer(rospy.Duration(0.1), self.update)
        self._running = False
        self.pub_player = rospy.Publisher('Player', Point, queue_size=1)
        self.pub_goal   = rospy.Publisher('AtGoal', Bool, queue_size=1)
        self.pub_start  = rospy.Publisher('AtStart', Bool, queue_size=1)
        self._odom_list = tf.TransformListener()
        self.player_rec = None
        self.player = Point()
        self.walls = []
        pygame.init()



    def on_init(self):

        print "here"
        self._running = True
        self.display_surf = pygame.display.set_mode((self.windowWidth, self.windowHeight))
        pygame.display.set_caption('Travel from Blue Square to Red Square')
        self._running = True
        self.N = self.maze.info.width  # number of rows
        self.M = self.maze.info.height  # number of columns
        self.maze_draw()
        self.player_draw()

        #start_loc_pixels_x = (start[0] * BLOCKSIZE_X) + math.floor(abs((BLOCKSIZE_X - PLAYERSIZE_X) * 0.5))
        #start_loc_pixels_y = (start[1] * BLOCKSIZE_Y) + math.floor(abs((BLOCKSIZE_Y - PLAYERSIZE_Y) * 0.5))
        pygame.display.update()



    def update(self,msg):
        """

        :return:
        """
        if self._running:
            self.display_surf.fill((0, 0, 0))
            self.walls = []
            self.maze_draw()
            if not self.check_collision():
                print "boom"
            self.player_draw()
            pygame.display.update()

            self.pub_player.publish(self.player)
            start = self.at_start()
            goal  = self.at_goal()



    def check_collision(self):
        print len(self.walls)
        for wall in self.walls:
            if self.player_rec.colliderect(wall):
                #print "player",self.player_rec
                #print "wall",wall
                return True
        print "not wall"
        return False

    def maze_callback(self,msg):
        self.maze = msg
        self.on_init()

    def invert(self):
        for index, row in enumerate(self.maze):
            self.maze[index] = row[::-1]

    def player_draw(self):
        arm_translation_x = 0.25
        arm_scale_x = 0.35
        arm_translation_y = 0.15
        arm_scale_y = 0.35

        self._odom_list.waitForTransform('base_link', 'master_EE', rospy.Time(0), rospy.Duration(1.0))
        (position, orientation) = self._odom_list.lookupTransform('base_link', 'master_EE', rospy.Time(0))

        EE_y = self.remap(position[1], -0.5, 0.5, 0, self.windowHeight)
        EE_x = self.remap(position[0], 0.15, 0.5, 0, self.windowWidth)



        (self.player.x, self.player.y) = (EE_y,EE_x)

        self.player_rec = pygame.Rect(self.player.x,self.player.y, PLAYERSIZE_X, PLAYERSIZE_Y)

        pygame.draw.rect(self.display_surf, GREEN,
                         (self.player.x,self.player.y, PLAYERSIZE_X, PLAYERSIZE_Y), 0)


    def remap(self,x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


    def maze_draw(self):
        # Iterate over maze

        for index, pt in enumerate(self.maze.data):
            bx,by = maze_helper.get_i_j(self.maze,index)
            self.walls.append(pygame.Rect(bx * BLOCKSIZE_X, by * BLOCKSIZE_Y, BLOCKSIZE_X, BLOCKSIZE_Y))
            cell = maze_helper.check_cell(self.maze,index)
            if cell == 1:
                pygame.draw.rect(self.display_surf, PURPLE,
                                 (bx * BLOCKSIZE_X, by * BLOCKSIZE_Y, BLOCKSIZE_X, BLOCKSIZE_Y), 0)
                self.walls.append(pygame.Rect(bx * BLOCKSIZE_X, by * BLOCKSIZE_Y, BLOCKSIZE_X, BLOCKSIZE_Y))

            elif cell == 2:
                pygame.draw.rect(self.display_surf, BLUE,
                                 (bx * BLOCKSIZE_X, by * BLOCKSIZE_Y, BLOCKSIZE_X, BLOCKSIZE_Y), 0)
                self.walls.append(pygame.Rect(bx * BLOCKSIZE_X, by * BLOCKSIZE_Y, BLOCKSIZE_X, BLOCKSIZE_Y))

            elif cell == 3:
                pygame.draw.rect(self.display_surf, RED,
                                 (bx * BLOCKSIZE_X, by * BLOCKSIZE_Y, BLOCKSIZE_X, BLOCKSIZE_Y), 0)
                self.walls.append(pygame.Rect(bx * BLOCKSIZE_X, by * BLOCKSIZE_Y, BLOCKSIZE_X, BLOCKSIZE_Y))




    def path_draw(self,path):
        """

        :param path:
        :return:
        """
        for point in path:

            pixels_x = (point.pose.position.x * 50) + math.floor(abs((50 - 20) * 0.5))
            #print pixels_x
            pixels_y = (point.pose.position.x * 50) + math.floor(abs((50 - 20) * 0.5))
            pygame.draw.rect(self.display_surf, GREEN,
                             (pixels_x, pixels_y, PLAYERSIZE_X, PLAYERSIZE_Y), 0)


    def at_start(self):

        start = maze_helper.getStart(self.maze)
        start_pixels = (start[0] * BLOCKSIZE_X, start[1] * BLOCKSIZE_Y)
        state = Bool()
        state.data = abs(self.player.x - start_pixels[0]) < PLAYERSIZE_X and \
                     abs(self.player.y - start_pixels[1]) < PLAYERSIZE_Y

        self.pub_start.publish(state)
        return state.data

    def at_goal(self):

        goal = maze_helper.getGoal(self.maze)
        goal_pixels = (goal[0] * BLOCKSIZE_X, goal[1] * BLOCKSIZE_Y)
        state = Bool()
        state.data = abs(self.player.x - goal_pixels[0]) < PLAYERSIZE_X and \
                    abs(self.player.y - goal_pixels[1]) < PLAYERSIZE_Y

        self.pub_goal.publish(state)
        return state.data


if __name__ == "__main__":

   game = Maze()
   while not rospy.is_shutdown():
       rospy.spin()
