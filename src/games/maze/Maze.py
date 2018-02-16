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
import tf
import tools.joint_states_listener
import tools.helper
import controllers.HapticController


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
BLOCKSIZE_X = 30
BLOCKSIZE_Y = 30
PLAYERSIZE_X = 10
PLAYERSIZE_Y = 10

# Translating arm motion to map
THRESHOLD = 0.05
Y_CUTOFF = 0.35



class Maze:
    windowWidth = 1000
    windowHeight = 600
    def __init__(self, maze_name="maze1"):
        """

        :param maze_name:
        """
        self.walls = []
        self.goal_rec = None
        self.start_rec = None
        self.player = Point()
        (self.player.x, self.player.y) =  (self.windowWidth*0.5,self.windowHeight*0.5)
        self.player_rec = pygame.Rect((self.player.x, self.player.y, PLAYERSIZE_X, PLAYERSIZE_Y) )
        self.solved_path = Path()
        self.wall_force = [0, 0, 0]
        rospy.init_node('MazeGame', anonymous=True)
        rospy.Subscriber("gen_maze", OccupancyGrid, self.maze_callback)
        rospy.Subscriber("a_star", Path, self.path_callback)
        rospy.Timer(rospy.Duration(0.1), self.update_GUI)
        self.running = False
        self.pub_player = rospy.Publisher('Player', Point, queue_size=1)
        self.pub_goal   = rospy.Publisher('AtGoal', Bool, queue_size=1)
        self.pub_start  = rospy.Publisher('AtStart', Bool, queue_size=1)
        self.pub_forces = rospy.Publisher("motors_server", WrenchStamped, queue_size=1)
        d_goal = 0.5*BLOCKSIZE_X + 0.5*PLAYERSIZE_X + 1.5*BLOCKSIZE_X
        d_obs = 0.5*BLOCKSIZE_X + 0.5*PLAYERSIZE_X + BLOCKSIZE_X
        self.controller = controllers.HapticController.HapticController(0.01,0.001,d_obs,d_goal)

        pygame.init()
        print("Ready to host maze")


    def on_init(self):
        """
        sets up the game
        :return:
        """
        self.display_surf = pygame.display.set_mode((self.windowWidth, self.windowHeight))
        self.running = True
        pygame.display.set_caption('Travel from Blue Square to Red Square')
        self.N = self.maze.info.height  # number of rows
        self.M = self.maze.info.width  # number of columns
        self.maze_draw()

        self.player_draw()

        #start_loc_pixels_x = (start[0] * BLOCKSIZE_X) + math.floor(abs((BLOCKSIZE_X - PLAYERSIZE_X) * 0.5))
        #start_loc_pixels_y = (start[1] * BLOCKSIZE_Y) + math.floor(abs((BLOCKSIZE_Y - PLAYERSIZE_Y) * 0.5))
        pygame.display.update()



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
            pygame.display.update()
            #self.pub_player.publish(self.player)
            #start = self.at_start()
            #goal  = self.at_goal()

    def maze_callback(self,msg):
        """
        saves the maze and sets up the game
        :param msg: occupany grid message
        :return:
        """
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
        (position, velocity, effort) = tools.helper.call_return_joint_states()
        # scales the input to the game
        EE_y = tools.helper.remap(position[0],-0.6,0.6,0,self.windowWidth )
        EE_x = tools.helper.remap(position[2],1.9,0.6,0,self.windowHeight )

        (self.player.x, self.player.y) =  (EE_y,EE_x)
        self.player_rec = pygame.Rect((EE_y, EE_x, PLAYERSIZE_X, PLAYERSIZE_Y) )
        player_center = Point()
        player_center.x = self.player_rec.centerx
        player_center.y = self.player_rec.centery
        wall_centers = self.check_collision_adaptive()
        goal_centers = self.goal_adaptive()
        self.controller.make_force(player_center,wall_centers,goal_centers)
        #[forces.wrench.force.x, forces.wrench.force.y, forces.wrench.force.z] = np.add(wall_force,goal_force)
        #self.check_collision(self.player)
        #self.pub_forces.publish(forces)


        # calls the joint state server
        #(position, velocity, effort) = self.call_return_joint_states(joint_names)

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
            #self.walls.append(pygame.Rect(bx * BLOCKSIZE_X, by * BLOCKSIZE_Y, BLOCKSIZE_X, BLOCKSIZE_Y))
            cell = maze_helper.check_cell(self.maze,index)
            if cell == 1:
                pygame.draw.rect(self.display_surf, PURPLE,
                                 (bx * BLOCKSIZE_X, by * BLOCKSIZE_Y, BLOCKSIZE_X, BLOCKSIZE_Y), 0)
                self.walls.append(pygame.Rect(bx * BLOCKSIZE_X, by * BLOCKSIZE_Y, BLOCKSIZE_X, BLOCKSIZE_Y))

            elif cell == 2:
                pygame.draw.rect(self.display_surf, BLUE,
                                 (bx * BLOCKSIZE_X, by * BLOCKSIZE_Y, BLOCKSIZE_X, BLOCKSIZE_Y), 0)
                #self.walls.append(pygame.Rect(bx * BLOCKSIZE_X, by * BLOCKSIZE_Y, BLOCKSIZE_X, BLOCKSIZE_Y))
                self.start_rec = pygame.Rect(bx * BLOCKSIZE_X, by * BLOCKSIZE_Y, BLOCKSIZE_X, BLOCKSIZE_Y)

            elif cell == 3:
                pygame.draw.rect(self.display_surf, RED,
                                 (bx * BLOCKSIZE_X, by * BLOCKSIZE_Y, BLOCKSIZE_X, BLOCKSIZE_Y), 0)
                self.goal_rec = pygame.Rect(bx * BLOCKSIZE_X, by * BLOCKSIZE_Y, BLOCKSIZE_X, BLOCKSIZE_Y)




    def path_callback(self,msg):
        """
        call_back for the optimal path
        sets the path
        :param msg: Path messaga
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
        """

        start = maze_helper.getStart(self.maze)
        start_pixels = (start[0] * BLOCKSIZE_X, start[1] * BLOCKSIZE_Y)
        state = Bool()
        state.data = abs(self.player.x - start_pixels[0]) < PLAYERSIZE_X and \
                     abs(self.player.y - start_pixels[1]) < PLAYERSIZE_Y

        self.pub_start.publish(state)
        return state.data

    def at_goal(self):
        """
        checks if we are at the goal
        :return: boolean check if we are at goal
        """
        goal = maze_helper.getGoal(self.maze)
        goal_pixels = (goal[0] * BLOCKSIZE_X, goal[1] * BLOCKSIZE_Y)
        state = Bool()
        state.data = abs(self.player.x - goal_pixels[0]) < PLAYERSIZE_X and \
                    abs(self.player.y - goal_pixels[1]) < PLAYERSIZE_Y
        self.pub_goal.publish(state)
        return state.data


    def check_collision_adaptive(self):
        walls = []
        centers = []

        player_x = math.floor(float(self.player_rec.centerx)/BLOCKSIZE_X) # This is the (x,y) block in the grid where the top left corner of the player is
        player_y = math.floor(float(self.player_rec.centery)/BLOCKSIZE_Y)
        for x in range(int(player_x) - 1, int(player_x) + 2):
            for y in range(int(player_y) - 1, int(player_y) + 2):
                point_index = maze_helper.index_to_cell(self.maze, x, y)
                if maze_helper.check_cell(self.maze, int(point_index)) == 1:
                    point = Point()
                    wall_block = pygame.Rect((x * BLOCKSIZE_X, y * BLOCKSIZE_Y, BLOCKSIZE_X, BLOCKSIZE_Y))
                    point.x = wall_block.centerx
                    point.y = wall_block.centery
                    centers.append(point)
                    walls.append(wall_block)
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
            print "have waypoint"
            for pose in self.solved_path.poses:
                pt = Point()

                pt.x = (pose.pose.position.x * BLOCKSIZE_X) + math.floor(abs((BLOCKSIZE_X - PLAYERSIZE_X) * 0.5))
                pt.y = (pose.pose.position.y * BLOCKSIZE_Y) + math.floor(abs((BLOCKSIZE_Y - PLAYERSIZE_Y) * 0.5))
                print pt
                points.append(pt)

        points.append(goal)

        return points




if __name__ == "__main__":

   game = Maze()
   while not rospy.is_shutdown():
       rospy.spin()
