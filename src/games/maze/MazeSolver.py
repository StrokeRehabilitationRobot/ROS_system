import Queue
import Maze
import pygame
import math
from pygame.locals import *


def a_star(maze):
    # find shortest path with fastest search
    start, goal = maze.getendpoints()
    frontier = Queue.PriorityQueue()
    frontier.put(start)
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0

    while not frontier.empty():
        current = frontier.get()
        # print "Visiting (%d, %d)" % (current[0], current[1])

        if current == goal:
            break

        for next in maze.neighbors_manhattan(current[0], current[1]):
            new_cost = cost_so_far[current] + costmove(current, next, came_from[current])
            if next not in cost_so_far:
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(goal, next)
                frontier.put(next, priority)
                came_from[next] = current
    return came_from

def costmove(current, next, prev):
    if (prev is None) or (next is None):
        return 1
    elif (next[0] == current[0] and current[0] == prev[0]) or (next[1] == current[1] and current[1] == prev[1]):
        print('straight')
        return 5
    else:
        print('turn')
        return 1

def priority_search(maze):
    # search best options in grid first
    start, goal = maze.getendpoints()
    frontier = Queue.PriorityQueue()
    frontier.put(start)
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0

    while not frontier.empty():
        current = frontier.get()
        #print "Visiting (%d, %d)" % (current[0], current[1])

        if current == goal:
            break

        for next in maze.neighbors_manhattan(current[0], current[1]):
            new_cost = cost_so_far[current] + 1
            if next not in cost_so_far:
                cost_so_far[next] = new_cost
                priority = new_cost
                frontier.put(next, priority)
                came_from[next] = current
    return came_from


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


maze = Maze.Maze()
maze.invert()
start, goal = maze.getendpoints()
player = Maze.Player()
pygame.init()
display_surf = pygame.display.set_mode((800,600))
pygame.display.set_caption('Pygame pythonspot.com example')
maze.draw(display_surf)
pygame.display.update()

came_from = a_star(maze)
print start
print came_from
path = reconstruct_path(came_from, start, goal)
print path

for position in path:
    pixels_x = (position[0] * 50) + math.floor(abs((50 - 20) * 0.5))
    #print pixels_x
    pixels_y = (position[1] * 50) + math.floor(abs((50 - 20) * 0.5))
    player.goTo(pixels_x, pixels_y)
    player.draw(display_surf)
pygame.display.update()
running = True
while running:
    pygame.event.pump()
    keys = pygame.key.get_pressed()
    if (keys[K_ESCAPE]):
        running = False
pygame.quit()
