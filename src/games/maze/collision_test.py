#!/usr/bin/env python
import pygame
from pygame.locals import *
import maze_helper
import mazeBank
from nav_msgs.msg import OccupancyGrid,Path
from geometry_msgs.msg import Pose,Point
import random

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

windowWidth = 800
windowHeight = 600

if __name__ == '__main__':

    # Get 2D Maze
    maze_2D = mazeBank.getmaze("maze2")
    row = len(maze_2D)
    col = len(maze_2D[0])
    data = [j for i in maze_2D for j in i]

    # Make it a 1D maze
    my_maze = OccupancyGrid()
    my_maze.data = data
    my_maze.info.width = col
    my_maze.info.height = row

    display_surf = pygame.display.set_mode((windowWidth, windowHeight))
    pygame.display.set_caption('Travel from Blue Square to Red Square')
    N = my_maze.info.height  # number of rows
    M = my_maze.info.width  # number of columns

    for index, pt in enumerate(my_maze.data):
        bx, by = maze_helper.get_i_j(my_maze, index)

        cell = maze_helper.check_cell(my_maze, index)
        if cell == 1:
            pygame.draw.rect(display_surf, PURPLE,
                             (bx * BLOCKSIZE_X, by * BLOCKSIZE_Y, BLOCKSIZE_X, BLOCKSIZE_Y), 0)
        elif cell == 2:
            pygame.draw.rect(display_surf, BLUE,
                             (bx * BLOCKSIZE_X, by * BLOCKSIZE_Y, BLOCKSIZE_X, BLOCKSIZE_Y), 0)
        elif cell == 3:
            pygame.draw.rect(display_surf, RED,
                             (bx * BLOCKSIZE_X, by * BLOCKSIZE_Y, BLOCKSIZE_X, BLOCKSIZE_Y), 0)


    player_topleft = Point()
    player_topleft.x = random.randint(1,windowWidth)
    player_topleft.y = random.randint(1,windowHeight)

    pygame.draw.rect(display_surf, GREEN,
                     (player_topleft.x, player_topleft.y, PLAYERSIZE_X, PLAYERSIZE_Y), 0)

    pygame.display.update()

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
    collisions = 4 * [False]
    for index, corner in enumerate(player_corners):
        cell_x = corner.x / BLOCKSIZE_X
        cell_y = corner.y / BLOCKSIZE_Y
        point_index = maze_helper.index_to_cell(my_maze, cell_x, cell_y)
        if maze_helper.check_cell(my_maze, point_index) == 1:
           collisions[index] = True

    print collisions

    if collisions[0] and collisions[1]:
        print("Go Down")
    elif collisions[1] and collisions[2]:
        print("Go Left")
    elif collisions[2] and collisions[3]:
        print("Go Up")
    elif collisions[3] and collisions[0]:
        print("Go Right")
    elif collisions[0] or collisions[1]:
        print("Go Down")
    elif collisions[2] or collisions[3]:
        print("Go Up")
    while True:
        pass