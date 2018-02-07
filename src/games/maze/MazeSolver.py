#!/usr/bin/env python

import Queue
import Maze
import pygame
import math
import maze_helper
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
import rospy
from pygame.locals import *


def a_star(maze):
    print("Called to solve")
    # find shortest path with fastest search
    path_pub = rospy.Publisher("a_star", Path, queue_size=1)
    start = maze_helper.getStart(maze) #column, row
    goal = maze_helper.getGoal(maze) #column, row
    frontier = Queue.PriorityQueue()
    frontier.put(start)
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0

    while not frontier.empty():
        current = frontier.get()
        for next in maze_helper.neighbors_manhattan(maze, current[0], current[1]):
            new_cost = cost_so_far[current] + costmove(current, next, came_from[current])
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(goal, next)
                frontier.put(next, priority)
                came_from[next] = current

    current = goal
    my_path = Path()
    this_step = PoseStamped()
    while current != start:
        this_step.pose.position.x = current[0]
        this_step.pose.position.y = current[1]
        this_step.pose.position.z = 0
        #this_step.header.stamp = rospy.Time.now()
        my_path.poses.append(this_step)
        current = came_from[current]
    my_path.poses.append(start) # optional
    my_path.poses.reverse() # optional
    #my_path.header.stamp = rospy.Time.now()
    print("Publishing")
    path_pub.publish(my_path)
    print("Published")

def costmove(current, next, prev):
    if (prev is None) or (next is None):
        return 1
    elif (next[0] == current[0] and current[0] == prev[0]) or (next[1] == current[1] and current[1] == prev[1]):
        return 5
    else:
        return 1

# def priority_search(maze):
#     # search best options in grid first
#     start, goal = maze.getendpoints()
#     frontier = Queue.PriorityQueue()
#     frontier.put(start)
#     came_from = {}
#     cost_so_far = {}
#     came_from[start] = None
#     cost_so_far[start] = 0
#
#     while not frontier.empty():
#         current = frontier.get()
#         #print "Visiting (%d, %d)" % (current[0], current[1])
#
#         if current == goal:
#             break
#
#         for next in maze.neighbors_manhattan(current[0], current[1]):
#             new_cost = cost_so_far[current] + 1
#             if next not in cost_so_far:
#                 cost_so_far[next] = new_cost
#                 priority = new_cost
#                 frontier.put(next, priority)
#                 came_from[next] = current
#     return came_from


def heuristic(a, b):
    (x1, y1) = a
    (x2, y2) = b
    return abs(x1 - x2) + abs(y1 - y2)


def reconstruct_path(came_from, start, goal):
    current = goal
    path = []
    while current != start:
        path.append(current)
        current = came_from[current]
    path.append(start) # optional
    path.reverse() # optional
    return path

if __name__ == "__main__":
    rospy.init_node('MazeSolver', anonymous=True)
    rospy.Subscriber("gen_maze", OccupancyGrid, a_star)
    print("Ready to solve")
    while not rospy.is_shutdown():
       rospy.spin()