#!/usr/bin/env python

import sys
import numpy as np
from strokeRehabSystem.srv import ReturnJointStates
from geometry_msgs.msg import Pose,Point, WrenchStamped
from std_msgs.msg import Bool
import rospy
import time
import tf
import tools.joint_states_listener
import tools.helper



def invert(self):
    for index, row in enumerate(self.maze):
        self.maze[index] = row[::-1]

def rec_to_point(rec):
    pt = Point()
    pt.x = rec.centerx
    pt.y = rec.centery
    return pt

def get_i_j(maze,index):
    """
    converts 1D to 2D
    :param maze: occupancy grid mesage
    :param index: index of 1D array
    :return: 2 ints of the the 2D array indexs
    """

    N = maze.info.width

    j = index/N
    i = index % N
    return i,j


def check_cell(maze, pt):
    """
    Check if a cell is a wall or the goal (1 = wall, 2 = goal, 0 = path)
    :param maze:occupany grid message
    :param pt:index to check
    :return:what is at that index
    """
    N = maze.info.width
    M = maze.info.height


    # If the cell in the maze array is a 1, the cell is a wall
    #if maze.data[pt] == 1:
        #return 1
    # If the cell in the maze array is a 2, the cell is the starting position
    if maze.data[pt] == 2:
        return 2
        # If the cell in the maze array is a 2, the cell is the starting position
    elif maze.data[pt] == 3:
        return 3
    elif maze.data[pt] == 0:
        return 0
    else:
        return 1

def getStart(maze):
    """
    get the stating location
    :param maze: occupany grid message
    :return: 2D index of the starting location
    """
    start = maze.data.index(2)
    return get_i_j(maze,start)

def getGoal(maze):
    """
    gets the goal location
    :param maze: occupany grid message
    :return: 2D index of the goal location
    """

    goal = maze.data.index(3)
    return get_i_j(maze,goal)

def construct_map(maze):
    pass

def index_to_cell(maze,x,y):
    """
    converts 2D index to 1D index
    :param maze: occupancy grid message
    :param x: x index of 2D array
    :param y: y index of 2D array
    :return: index of 1D array
    """
    if x < 0 or x >= maze.info.width or y < 0 or y >= maze.info.height:
        return maze.data.index(1)
    else:
        return maze.info.width*y + x

def joint_to_game(x_range, y_range  ):

    (position, velocity, effort) = tools.helper.call_return_joint_states()
    # scales the input to the game
    EE_y = tools.helper.remap(round(position[0],5),-0.6,0.6,x_range[0],x_range[1] )
    EE_x = tools.helper.remap(round(position[2],5),1.9,0.6,y_range[0],y_range[1])
    #EE_x = remap(position[1],-0.95,0.35,y_range[0],y_range[1])
    return (EE_y,EE_x)

def task_to_game(odom_list,x_range, y_range):


    odom_list.waitForTransform('base_link', 'master_EE', rospy.Time(0),rospy.Duration(0.1))
    (position, _ ) = odom_list.lookupTransform('base_link', 'master_EE', rospy.Time(0))
    EE_y = tools.helper.remap(position[1],-0.20,0.20,x_range[0],x_range[1] )
    EE_x = tools.helper.remap(position[2],0.15,-0.15,y_range[0],y_range[1])

    return (EE_y,EE_x)

def neighbors_manhattan(maze,loc_x, loc_y):

    neighbors_in = [(loc_x - 1, loc_y), (loc_x, loc_y + 1), (loc_x + 1, loc_y), (loc_x, loc_y - 1)]
    neighbors_out = []
    for option in neighbors_in:
        #print "checking point: ", index_to_cell(maze, option[0], option[1])
        #print "cell ID: ", check_cell(maze, index_to_cell(maze, option[0], option[1]))
        if check_cell(maze, index_to_cell(maze, option[0], option[1])) in (0, 2, 3):
            neighbors_out.append(option)

    return neighbors_out
