#!/usr/bin/env python

import sys
import pygame
from pygame.locals import *
import maze_helper
import math
import threading
from nav_msgs.msg import OccupancyGrid,Path
from strokeRehabSystem.msg import hapticForce
from geometry_msgs.msg import Pose,Point, WrenchStamped
from std_msgs.msg import Bool
import rospy
from strokeRehabSystem.msg import *

import tf

import time


class Maze:

    def __init__(self, maze_name="maze1"):

        self.goal_rec = None
        self.start_rec = None
        self.player = Rect((maze_helper.WINDOWWIDTH * 0.5, maze_helper.WINDOWHEIGHT * 0.5, maze_helper.PLAYERSIZE_X, maze_helper.PLAYERSIZE_Y))

        self.solved_path = []
        self.score = 0
        self.game_timer = 0

        self.running = False
        self.am_i_at_goal = False
        self.am_i_at_start = False
        self.time0 = time.clock()
        self.csv = open("/home/cibr-strokerehab/Documents/JointStatesRecording.csv", "w")

        rospy.init_node('MazeGame', anonymous=True)
        rospy.Subscriber("gen_maze", OccupancyGrid, self.maze_callback)
        rospy.Subscriber("a_star", Path, self.path_callback)
        rospy.Subscriber('Player_State', PlayerState, self.update_player)

        self.odom_list = tf.TransformListener()
        self.pub_goal = rospy.Publisher('at_goal', Bool, queue_size=1)
        self.pub_start = rospy.Publisher('at_start', Bool, queue_size=1)
        self.pub_enviroment = rospy.Publisher("enviroment", hapticForce, queue_size=1)

        force_thread = threading.Thread(target=self.update_force)
        force_thread.daemon = True
        force_thread.start()
        rospy.Timer(rospy.Duration(0.1), self.update_GUI)

        pygame.init()
        pygame.font.init()



    def on_init(self):
        """
        sets up the game, called from maze_callback when a new maze is published.
        :return:
        """
        self.display_surf = pygame.display.set_mode((maze_helper.WINDOWWIDTH, maze_helper.WINDOWHEIGHT))

        self.running = True
        self.am_i_at_goal = False
        self.am_i_at_start = False
        self.score = 0
        self.player = pygame.Rect( 0.5*maze_helper.WINDOWHEIGHT, 0.5*maze_helper.WINDOWHEIGHT, maze_helper.PLAYERSIZE_X, maze_helper.PLAYERSIZE_Y )
        self.game_timer = time.time()
        pygame.display.set_caption('Travel from Blue Square to Red Square')
        self.maze_draw()
        self.player_draw()
        pygame.display.update()

    def update_player(self, msg):
        """
        call_back when the player's location has changed, based on end-effector movement. The player's location in the
        task space, where (0,0) is in the middle of the task space, the x-axis is positive to the right, and the y-axis
        is positive going up, is converted to a location in the game, where (0,0) is in the top left corner, x-axis
        extends right, and y-axis points down.
        :param msg: x and y location of the player in the end effector space (actually x and z location in robot frame)
        :return:
        """
        (x, y) = maze_helper.task_to_game(msg.pose.position.x, msg.pose.position.y)
        self.player.center = (x,y)

    def update_force(self):
        """
        This function runs separately, in its own thread, until the calling thread is killed.
        :return:
        """
        while 1:

            msg = hapticForce()
            if self.running:

                centers = maze_helper.collision_plane(self.maze,self.player)
                task_coor = []

                for center in centers:

                    pt = Point()
                    x = center.x - 0.5*maze_helper.PLAYERSIZE_X
                    y = center.y - 0.5*maze_helper.PLAYERSIZE_Y
                    (pt.x,pt.y) = maze_helper.game_to_task(center.x,center.y)
                    task_coor.append(pt)
                    temp = pygame.Rect((x,y), (maze_helper.PLAYERSIZE_X,maze_helper.PLAYERSIZE_Y) )
                    pygame.draw.rect(self.display_surf, maze_helper.GREEN , temp, 0)

                msg.obstacles = task_coor
                #msg.goals = maze_helper.goal_adaptive(self.start_rec,self.goal_rec,self.path_draw)
                self.pub_enviroment.publish(msg)
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
                # Checking to see if maze has been completed, and continues timer
                self.am_i_at_goal = self.at_goal()
                elapsed_time = time.time() - self.game_timer
            else:
                # Checking to see if the maze has been started, keeps timer at 0
                self.am_i_at_start = self.at_start()
                elapsed_time = 0

            self.update_score()
            scoretext = "Current Score: %d, Time Elapsed: %d s" %(self.score, elapsed_time)
            self.myfont = pygame.font.SysFont('Comic Sans MS', 18)
            textsurface = self.myfont.render(scoretext, False, maze_helper.WHITE)
            self.display_surf.blit(textsurface, (0,0))
            pygame.display.update()


    def maze_callback(self,msg):
        """
        Called when a new maze is published, identifies maze components and calls initialization function
        :param msg: occupany grid message
        :return:
        """
        self.maze = msg
        self.walls = []
        starts = []
        goals = []
        for index, pt in enumerate(self.maze.data):

            bx,by = maze_helper.get_i_j(self.maze,index)
            cell = maze_helper.check_cell(self.maze,index)

            if cell == 1:
                self.walls.append(pygame.Rect(bx * maze_helper.BLOCKSIZE_X, by * maze_helper.BLOCKSIZE_Y, maze_helper.BLOCKSIZE_X, maze_helper.BLOCKSIZE_Y))
            elif cell == 2:
                starts.append(pygame.Rect(bx * maze_helper.BLOCKSIZE_X, by * maze_helper.BLOCKSIZE_Y, maze_helper.BLOCKSIZE_X, maze_helper.BLOCKSIZE_Y))
                self.start_rec = starts[0].unionall(starts)
            elif cell == 3:
                goals.append(pygame.Rect(bx * maze_helper.BLOCKSIZE_X, by * maze_helper.BLOCKSIZE_Y, maze_helper.BLOCKSIZE_X, maze_helper.BLOCKSIZE_Y))
                self.goal_rec = goals[0].unionall(goals)

        self.on_init()


    def path_callback(self,msg):
        """
        call_back for the the solved path through the maze. Each point in the solution is represented as a player-sized
        block in the center of a maze block
        :param msg: Path message
        :return:
        """

        path = []
        for point in msg.poses:

            pixels_x = (point.pose.position.x * maze_helper.BLOCKSIZE_X) + math.floor(abs((maze_helper.BLOCKSIZE_X - maze_helper.PLAYERSIZE_X) * 0.5))
            pixels_y = (point.pose.position.y * maze_helper.BLOCKSIZE_Y) + math.floor(abs((maze_helper.BLOCKSIZE_Y - maze_helper.PLAYERSIZE_Y) * 0.5))
            path.append(pygame.Rect(pixels_x, pixels_y, maze_helper.PLAYERSIZE_X, maze_helper.PLAYERSIZE_Y))

        self.solved_path = path



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
        # if self.am_i_at_start:
        #     self.update_score()
        #     print "Score:", self.score
        pygame.draw.ellipse(self.display_surf, maze_helper.WHITE, self.player, 0)

    def maze_draw(self):
        """
        callback for the maze
        draws the maze
        :return: none
        """
        for wall in self.walls:
                pygame.draw.rect(self.display_surf, maze_helper.PURPLE, wall, 0)

        pygame.draw.rect(self.display_surf, maze_helper.RED, self.goal_rec, 0)
        pygame.draw.rect(self.display_surf, maze_helper.BLUE, self.start_rec, 0)

    def at_start(self):
        """
        checks if we are at the starting location, and if so, starts timer
        :return: boolean check if we are at the starting location

        """
        state = Bool()
        state.data = self.start_rec.contains(self.player)

        if state.data:
            self.pub_start.publish(state)
            self.game_timer = time.time()

        return state.data

    def at_goal(self):
        """
        checks if we are at the goal, and if so, publishes a flag to generate a new maze, calculates the score, and
        begins the reset process.
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


    def update_score(self, assistance=3):

        player_x = math.floor(float(self.player.centerx) / maze_helper.BLOCKSIZE_X)  # This is the (x,y) block in the grid where the center of the player is
        player_y = math.floor(float(self.player.centery) / maze_helper.BLOCKSIZE_Y)
        point_index = maze_helper.index_to_cell(self.maze, player_x, player_y)

        if maze_helper.check_cell(self.maze, int(point_index)) == 1:
            self.score -= 1

        for rec in self.solved_path:
            if rec.centerx == player_x and rec.centery == player_y:
                #print "On track"
                reward = math.floor(1./len(self.solved_path) * 100)
                self.score += reward
            # else:
            #     print "Player (x,y):", player_x, player_y
            #     print "Waypoint (x,y):", pose.pose.position.x, pose.pose.position.y



if __name__ == "__main__":

    game = Maze()
    while not rospy.is_shutdown():
        rospy.spin()
