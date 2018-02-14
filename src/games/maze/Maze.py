#!/usr/bin/env python
import pygame
from pygame.locals import *
import maze_helper
import math
import mazeBank

import numpy
import sys
from nav_msgs.msg import OccupancyGrid,Path
from strokeRehabSystem.srv import ReturnJointStates
from geometry_msgs.msg import Pose,Point, WrenchStamped
from std_msgs.msg import Bool
import rospy
import tf
import tools.joint_states_listener
import tools.helper

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
BLOCKSIZE_X = 20
BLOCKSIZE_Y = 20
PLAYERSIZE_X = 8
PLAYERSIZE_Y = 8

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
        self.walls = []
        self.player = Point()
        (self.player.x, self.player.y) =  (self.windowWidth*0.5,self.windowHeight*0.5)
        self.player_rec = pygame.Rect((self.player.x, self.player.y, PLAYERSIZE_X, PLAYERSIZE_Y) )
        self.solved_path = Path()
        self.wall_force = [0, 0, 0]
        rospy.init_node('MazeGame', anonymous=True)
        rospy.Subscriber("gen_maze", OccupancyGrid, self.maze_callback)
        rospy.Subscriber("a_star", Path, self.path_callback)
        rospy.Timer(rospy.Duration(0.01), self.update_feedback)
        rospy.Timer(rospy.Duration(0.1), self.update_GUI)
        self.running = False
        self.pub_player = rospy.Publisher('Player', Point, queue_size=1)
        self.pub_goal   = rospy.Publisher('AtGoal', Bool, queue_size=1)
        self.pub_start  = rospy.Publisher('AtStart', Bool, queue_size=1)
        self.pub_forces = rospy.Publisher("motors_server", WrenchStamped, queue_size=1)
        self._odom_list = tf.TransformListener()

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
            self.pub_player.publish(self.player)
            start = self.at_start()
            goal  = self.at_goal()

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

            elif cell == 3:
                pygame.draw.rect(self.display_surf, RED,
                                 (bx * BLOCKSIZE_X, by * BLOCKSIZE_Y, BLOCKSIZE_X, BLOCKSIZE_Y), 0)
                #self.walls.append(pygame.Rect(bx * BLOCKSIZE_X, by * BLOCKSIZE_Y, BLOCKSIZE_X, BLOCKSIZE_Y))




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

            pixels_x = (point.pose.position.x * 50) + math.floor(abs((50 - 20) * 0.5))
            pixels_y = (point.pose.position.y * 50) + math.floor(abs((50 - 20) * 0.5))
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




    def update_feedback(self,msg):
        (position, velocity, effort) = tools.helper.call_return_joint_states()
        # scales the input to the game
        EE_y = tools.helper.remap(position[0],-0.6,0.6,0,self.windowWidth )
        EE_x = tools.helper.remap(position[2],2.1,0.6,0,self.windowHeight )
        (self.player.x, self.player.y) =  (EE_y,EE_x)
        self.player_rec = pygame.Rect((EE_y, EE_x, PLAYERSIZE_X, PLAYERSIZE_Y) )
        forces = WrenchStamped()
        forces.header.frame_id = "master"
        [forces.wrench.force.x, forces.wrench.force.y, forces.wrench.force.z] = self.check_collision(self.player)
        #self.check_collision(self.player)
        self.pub_forces.publish(forces)

    def check_collision(self, point):
        """
        checks if player (top left corner represented by point passed in)
        :return: 3D vector for joint torques
        """
        #player_left = self.player_rec.centerx - 0.5*BLOCKSIZE_X
        #player_top  = self.player_rec.centery  + 0.5*BLOCKSIZE_Y

        #inflated_player = pygame.Rect( (player_left,player_top,BLOCKSIZE_X,BLOCKSIZE_Y) )

        hit = self.player_rec.collidelist(self.walls)

        if hit > -1:
            pygame.draw.rect(self.display_surf, GREEN , self.walls[hit], 0)

        # Get four corners of the player (based on top left corner passed in)
        player_topleft = Point()
        player_topleft.x = math.floor(float(point.x)/BLOCKSIZE_X) # This is the (x,y) block in the grid where the top left corner of the player is
        player_topleft.y = math.floor(float(point.y)/BLOCKSIZE_Y)
        #print "x", int(player_topleft.x)
        #print "y",int(player_topleft.y)

        player_topright = Point()
        player_topright.x = math.floor(float(point.x + PLAYERSIZE_X)/BLOCKSIZE_X)
        player_topright.y = player_topleft.y#/BLOCKSIZE_Y

        player_bottomright   = Point()
        player_bottomright.x = math.floor(float(point.x + PLAYERSIZE_X)/BLOCKSIZE_X)
        player_bottomright.y = math.floor(float(point.y + PLAYERSIZE_Y)/BLOCKSIZE_Y)

        player_bottomleft   = Point()
        player_bottomleft.x = player_topleft.x#/BLOCKSIZE_X
        player_bottomleft.y = math.floor(float(point.y + PLAYERSIZE_Y)/BLOCKSIZE_Y)

        player_corners = [player_topleft, player_topright, player_bottomright, player_bottomleft]
        # Check for each corner in a block labeled as a wall
        collisions = 4 * [False]
        for index, corner in enumerate(player_corners):
            point_index = maze_helper.index_to_cell(self.maze, corner.x, corner.y)
            if maze_helper.check_cell(self.maze, int(point_index)) == 1:
                collisions[index] = True

        print collisions

        # Return force vector in three dimensions
        # TODO: Can we assign a force in two directions for each corner, so that they add together if multiple corners are hit?
        # TODO: That would be easier if the direction of the vectors were known. Test the below first.
        mag = 0.5
        if all(collisions):
            print("Lost in the walls")
            #self.wall_force = [0, 0, 0]
        else:
            if collisions[0] and collisions[1]:
                self.wall_force = [0, 0, -mag]
                print("Go Down")
            elif collisions[1] and collisions[2]:
                self.wall_force = [-mag, 0, 0]
                print("Go Left")
            elif collisions[2] and collisions[3]:
                self.wall_force = [0, 0, mag]
                print("Go Up")
            elif collisions[3] and collisions[0]:
                self.wall_force = [mag, 0, 0]
                print("Go Right")
            else:
                self.wall_force = [0, 0, 0]
                print("All clear")

        return self.wall_force


if __name__ == "__main__":

   game = Maze()
   while not rospy.is_shutdown():
       rospy.spin()
