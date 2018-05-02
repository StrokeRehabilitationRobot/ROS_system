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
    # find shortest path with fastest search
    start = maze_helper.get_start(maze) #column, row
    goal = maze_helper.get_goal(maze) #column, row
    print "start", start
    print  "goal", goal
    print "width", maze.info.width
    print "height", maze.info.height
    frontier = Queue.PriorityQueue()
    frontier.put(start)
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0
    print "solving"

    while not frontier.empty():
        current = frontier.get()

        for next in maze_helper.neighbors_manhattan(maze, current[0], current[1]):
            new_cost = cost_so_far[current] + costmove(maze, current, next, came_from[current])
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(goal, next)
                frontier.put(next, priority)
                came_from[next] = current

    current = goal
    my_path = Path()
    step_index = 0
    print "rebuilding path"
    while current != start:
        my_path.poses.append(PoseStamped())
        my_path.poses[step_index].pose.position.x = current[0]
        my_path.poses[step_index].pose.position.y = current[1]
        my_path.poses[step_index].pose.position.z = 0
        #this_step.header.stamp = rospy.Time.now()
        current = came_from[current]
        step_index += 1
    my_path.poses.append(PoseStamped())
    my_path.poses[step_index].pose.position.x = start[0]
    my_path.poses[step_index].pose.position.y = start[1]
    my_path.poses[step_index].pose.position.z = 0
    my_path.poses.reverse()
    print("solved")
    return my_path

def break_into_segments(my_path):
    # Break path into segments
    segment_i = 0
    segments = [[]]
    # print my_path.poses
    for index, step in enumerate(my_path.poses):
        if index == 0:
            prev = step
        else:
            prev = my_path.poses[index - 1]

        try:
            next = my_path.poses[index + 1]
        except IndexError:
            next = step

        #print("Prev: (%d, %d), Step: (%d, %d), Next: (%d, %d)") \
             #% (prev.pose.position.x, prev.pose.position.y, step.pose.position.x, step.pose.position.y,
                #next.pose.position.x, next.pose.position.y)

        if (next.pose.position.x == step.pose.position.x and step.pose.position.x == prev.pose.position.x) \
                or (next.pose.position.y == step.pose.position.y and step.pose.position.y == prev.pose.position.y):
            #print("straight")
            segments[segment_i].append(step)
        else:
            #print("turn")
            if segments[segment_i] != []:
                segments.append([])
                segment_i += 1
            segments[segment_i].append(prev)
            segments[segment_i].append(step)
            segments[segment_i].append(next)
            segments.append([])
            segment_i += 1
    return segments

def costmove(maze, current, next, prev):
    cost = 0
    if (prev is None) or (next is None):
        cost += 1
    elif (next[0] == current[0] and current[0] == prev[0]) or (next[1] == current[1] and current[1] == prev[1]):
        cost += 1
    else:
        cost += 5

    num_neighbors = len(maze_helper.neighbors_manhattan(maze, next[0], next[1]))
    cost = cost + 16 - (4*num_neighbors) # add cost for every wall neighbor

    return cost
def break_into_lean_segments(my_path):
    # Break path into segments
    segments = Path()

    # print my_path.poses
    for index, step in enumerate(my_path.poses):
        if index == 0:
            prev = step
        else:
            prev = my_path.poses[index - 1]
        try:
            next = my_path.poses[index + 1]
        except IndexError:
            next = step

        if not ((next.pose.position.x == step.pose.position.x and step.pose.position.x == prev.pose.position.x) \
                or (next.pose.position.y == step.pose.position.y and step.pose.position.y == prev.pose.position.y)):
            segments.poses.append(prev)
            segments.poses.append(next)

    return segments

def get_attractor_list(segments, assistance):
    attractors = Path()
    for segment in segments:
        if len(segment) <=2:
            continue
        elif segment[0].pose.position.x != segment[2].pose.position.x and segment[0].pose.position.y != segment[2].pose.position.y:
            attractors.poses.append(segment[0])
            attractors.poses.append(segment[2])
        else:
            step_size = int(max(math.floor(float(len(segment)/assistance)), 1))
            attractors.poses.extend(segment[::step_size])

    return attractors


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

def solve_into_segments(maze):
    print("called")
    path_pub = rospy.Publisher("a_star", Path, queue_size=1,latch=True)
    solution = a_star(maze) # returns the path of the complete solution
    segments = break_into_segments(solution) # returns a list of paths, where each path is a straight or turn
    attractors = get_attractor_list(segments, assistance=1) # turn list of paths into one list of points representing solution
    lean_segments = break_into_lean_segments(solution) # returns a path of waypoints
    path_pub.publish(attractors)
    #path_pub.publish(lean_segments)

if __name__ == "__main__":
    rospy.init_node('MazeSolver', anonymous=True)
    rospy.Subscriber("gen_maze", OccupancyGrid, solve_into_segments)
    print("Ready to solve")
    while not rospy.is_shutdown():
       rospy.spin()
