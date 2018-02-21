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
from strokeRehabSystem.msg import hapticForce
from geometry_msgs.msg import Pose,Point, WrenchStamped
from std_msgs.msg import Bool
import rospy
import time
import tf
import tools.joint_states_listener
import tools.helper
import controllers.HapticController
import EnviromentDynamics
from operator import sub
import time

# Colors for use throughout


# Map element sizes


class Maze:

    windowWidth = 1000
    windowHeight = 600

    def __init__(self, maze_name="maze1"):
        """

        :param maze_name:
        """

        self.walls = []
        self.starts = []
        self.goals = []
        self.goal_rec = None
        self.start_rec = None
        # AV_TODO: can use player = player_rec?

        self.player = Rect((self.windowWidth*0.5, self.windowHeight*0.5, maze_helper.PLAYERSIZE_X, maze_helper.PLAYERSIZE_Y) )

        self.solved_path = []
        self.score = 0
        self.wall_force = [0, 0, 0]
        self.game_timer = 0

        self.running = False
        self.am_i_at_goal = False
        self.am_i_at_start = False
        self.time0 = 0
        self.pose_old = (0,0)

        rospy.init_node('MazeGame', anonymous=True)
        rospy.Subscriber("gen_maze", OccupancyGrid, self.maze_callback)
        rospy.Subscriber("a_star", Path, self.path_callback)
        rospy.Timer(rospy.Duration(0.1), self.update_GUI)
        rospy.Timer(rospy.Duration(0.01), self.update_player)
        self.pub_player = rospy.Publisher('Player', Point, queue_size=1)
        self.pub_goal   = rospy.Publisher('at_goal', Bool, queue_size=1)
        self.pub_start  = rospy.Publisher('at_start', Bool, queue_size=1)
        self.pub_forces = rospy.Publisher("motors_server", WrenchStamped, queue_size=1)
        self.pub_forces = rospy.Publisher("haptic", hapticForce, queue_size=1)
        .
        d_goal = 0.5*maze_helper.BLOCKSIZE_X + 0.5*maze_helper.PLAYERSIZE_X + 1.5*maze_helper.BLOCKSIZE_X
        d_obs = 0.5*maze_helper.BLOCKSIZE_X + 0.5*maze_helper.PLAYERSIZE_X + maze_helper.BLOCKSIZE_X

        self.controller = EnviromentDynamics.EnviromentDynamics(0.01,0.001,0.0001,0.0001,d_obs,d_goal)
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
        self.starts = []
        self.goals = []

        self.game_timer = time.time()
        pygame.display.set_caption('Travel from Blue Square to Red Square')
        self.maze_draw()
        self.player_draw()
        pygame.display.update()

        self.pose_old = self.player
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
                elapsed_time = time.time() - self.game_timer
            else:
                self.am_i_at_start = self.at_start()
                elapsed_time = 0

            self.player_draw()
            scoretext = "Current Score: %d, Time Elapsed: %d s" %(self.score, elapsed_time)
            textsurface = self.myfont.render(scoretext, False, maze_helper.WHITE)
            self.display_surf.blit(textsurface, (0,0))
            pygame.display.update()


    def maze_callback(self,msg):
        """
        saves the maze and sets up the game
        :param msg: occupany grid message
        :return:
        """
        self.maze = msg

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

        self.walls = np.asarray(self.walls)
        self.on_init()

    def player_draw(self):
        """
        draws the player location
        :return:
        """
        player_center = Point()
        player_center.x = self.player.centerx
        player_center.y = self.player.centery
        wall_centers = self.check_collision_adaptive()
        goal_centers = self.goal_adaptive()

        if self.am_i_at_start:
            self.update_score()
            print "Score:", self.score
            self.controller.make_force(player_center,v,wall_centers,goal_centers)

        pygame.draw.rect(self.display_surf, maze_helper.WHITE,self.player, 0)

    def update_player(self,msg):
        (x, y) =  maze_helper.joint_to_game((0,self.windowWidth), (0,self.windowHeight) )
        v = 0#self.get_velocity()
        self.player = pygame.Rect((x, y, maze_helper.PLAYERSIZE_X, maze_helper.PLAYERSIZE_Y) )

    def get_velocity(self):
        dt = (time.time() - self.time0)
        v = tuple(map(sub, (self.player.centerx, self.player.centery) , self.pose_old))
        self.pose_old = (self.player.centerx, self.player.centery)
        v = (v[0]/dt,v[1]/dt)
        self.time0 = time.time()
        return v


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


    def path_draw(self):
        """
        draws the path
        :return:
        """

        for rect in self.solved_path:
            pygame.draw.rect(self.display_surf, GREEN,rect, 0)

    def at_start(self):
        """
        checks if we are at the starting location
        :return: boolean check if we are at the starting location

        """
        state = Bool()
        state.data = self.start_rec.contains(self.player)
        if state.data:
            self.game_timer = time.time()
        #self.pub_start.publish(state)
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

    def check_collision_adaptive(self):
        #walls = []
        centers = []
        player_x = math.floor(float(self.player.centerx)/maze_helper.BLOCKSIZE_X) # This is the (x,y) block in the grid where the top left corner of the player is
        player_y = math.floor(float(self.player.centery)/maze_helper.BLOCKSIZE_Y)
        for x in range(int(player_x) - 1, int(player_x) + 2):
            for y in range(int(player_y) - 1, int(player_y) + 2):
                point_index = maze_helper.index_to_cell(self.maze, x, y)
                neighbors = maze_helper.neighbors_manhattan(self.maze, x,y)
                #goal = maze_helper.getGoal(self.maze)
                near_goal = [block for block in self.goals if block in neighbors]
                if maze_helper.check_cell(self.maze, int(point_index)) == 1 and not near_goal:
                    point = Point()
                    wall_block = pygame.Rect((x * maze_helper.BLOCKSIZE_X, y * maze_helper.BLOCKSIZE_Y, maze_helper.BLOCKSIZE_X, maze_helper.BLOCKSIZE_Y))
                    point.x = wall_block.centerx
                    point.y = wall_block.centery
                    centers.append(point)
                    #walls.append(wall_block)
                    pygame.draw.rect(self.display_surf, GREEN , wall_block, 0)

        pygame.display.update()
        return centers

    def goal_adaptive(self):

        points = []
        goal = maze_helper.rec_to_point(self.goal_rec)
        start = maze_helper.rec_to_point(self.start_rec)
        points.append(goal)
        points.append(start)

        if self.solved_path:
            for rec in self.solved_path:
                pt = Point()
                pt.x = (rec.centerx * maze_helper.BLOCKSIZE_X) + math.floor(abs((maze_helper.BLOCKSIZE_X - maze_helper.PLAYERSIZE_X) * 0.5))
                pt.y = (rec.centery * maze_helper.BLOCKSIZE_Y) + math.floor(abs((maze_helper.BLOCKSIZE_Y - maze_helper.PLAYERSIZE_Y) * 0.5))
                points.append(pt)

        points.append(goal)

        return points

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
