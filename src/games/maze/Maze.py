#!/usr/bin/env python
import pygame
from pygame.locals import *
import maze_helper
import math
import mazeBank
import Player
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
        rospy.Subscriber("a_star", Path, self.path_callback)
        rospy.Timer(rospy.Duration(0.1), self.update)
        self.running = False
        self.pub_player = rospy.Publisher('Player', Point, queue_size=1)
        self.pub_goal   = rospy.Publisher('AtGoal', Bool, queue_size=1)
        self.pub_start  = rospy.Publisher('AtStart', Bool, queue_size=1)
        self.pub_forces = rospy.Publisher("motors_server", WrenchStamped, queue_size=1)
        self._odom_list = tf.TransformListener()
        self.player = Point()
        self.solved_path = Path()
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
        #start_loc_pixels_x = (start[0] * BLOCKSIZE_X) + math.floor(abs((BLOCKSIZE_X - PLAYERSIZE_X) * 0.5))
        #start_loc_pixels_y = (start[1] * BLOCKSIZE_Y) + math.floor(abs((BLOCKSIZE_Y - PLAYERSIZE_Y) * 0.5))
        pygame.display.update()


    def update(self,msg):
        """
        refeshs the game on a timer callback
        :msg: not used
        :return:
        """

        if self.running:
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

        # joint names



        # calls the joint state server
        #(position, velocity, effort) = self.call_return_joint_states(joint_names)
        (position, velocity, effort) = tools.helper.call_return_joint_states()
        # scales the input to the game
        EE_y = tools.helper.remap(position[0],-0.6,0.6,0,self.windowWidth )
        EE_x = tools.helper.remap(position[2],2.1,0.6,0,self.windowHeight )
        (self.player.x, self.player.y) = numpy.multiply([EE_x, EE_y], [BLOCKSIZE_X, BLOCKSIZE_Y])
        forces = WrenchStamped()
        forces.header.frame_id = "master"
        forces.wrench.force = self.check_collision(self.player)
        self.pub_forces.publish(forces)
        pygame.draw.rect(self.display_surf, WHITE,
                         (EE_y, EE_x, PLAYERSIZE_X, PLAYERSIZE_Y), 0)

    #
    # def call_return_joint_states(self,joint_names):
    #     """
    #     joint server callback, collects the joint angles
    #     :param joint_names: name of joints
    #     :return:
    #     """
    #     rospy.wait_for_service("return_joint_states")
    #     try:
    #         s = rospy.ServiceProxy("return_joint_states", ReturnJointStates)
    #         resp = s(joint_names)
    #     except rospy.ServiceException, e:
    #         print "error when calling return_joint_states: %s"%e
    #         sys.exit(1)
    #     for (ind, joint_name) in enumerate(joint_names):
    #         if(not resp.found[ind]):
    #             print "joint %s not found!"%joint_name
    #     return (resp.position, resp.velocity, resp.effort)


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
            elif cell == 2:
                pygame.draw.rect(self.display_surf, BLUE,
                                 (bx * BLOCKSIZE_X, by * BLOCKSIZE_Y, BLOCKSIZE_X, BLOCKSIZE_Y), 0)
            elif cell == 3:
                pygame.draw.rect(self.display_surf, RED,
                                 (bx * BLOCKSIZE_X, by * BLOCKSIZE_Y, BLOCKSIZE_X, BLOCKSIZE_Y), 0)

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

    def check_collision(self, point):
        """
        checks if player (top left corner represented by point passed in)
        :return: 3D vector for joint torques
        """

        # Get four corners of the player (based on top left corner passed in)
        player_topleft = Point()
        player_topleft.x = point.x
        player_topleft.y = point.y

        player_topright = Point()
        player_topright.x = player_topleft.x + PLAYERSIZE_X
        player_topright.y = player_topleft.y

        player_bottomright = Point()
        player_bottomright.x = player_topleft.x + PLAYERSIZE_X
        player_bottomright.y = player_topleft.y + PLAYERSIZE_Y

        player_bottomleft = Point()
        player_bottomleft.x = player_topleft.x
        player_bottomleft.y = player_topleft.y + PLAYERSIZE_Y

        player_corners = [player_topleft, player_topright, player_bottomright, player_bottomleft]

        # Check for each corner in a block labeled as a wall
        collisions = 4 * [False]
        for index, corner in enumerate(player_corners):
            cell_x = corner.x / BLOCKSIZE_X
            cell_y = corner.y / BLOCKSIZE_Y
            point_index = maze_helper.index_to_cell(self.maze, cell_x, cell_y)
            if maze_helper.check_cell(self.maze, point_index) == 1:
                collisions[index] = True

        # Return force vector in three dimensions
        if collisions[0] and collisions[1]:
            return [1, 0, 0]
            print("Go Down")
        elif collisions[1] and collisions[2]:
            return [0, -1, 0]
            print("Go Left")
        elif collisions[2] and collisions[3]:
            return [-1, 0, 0]
            print("Go Up")
        elif collisions[3] and collisions[0]:
            return [0, 1, 0]
            print("Go Right")
        elif collisions[0] or collisions[1]:
            return [1, 0, 0]
            print("Go Down")
        elif collisions[2] or collisions[3]:
            return [-1, 0, 0]
            print("Go Up")


if __name__ == "__main__":

   game = Maze()
   while not rospy.is_shutdown():
       rospy.spin()
