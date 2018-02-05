
import pygame
from pygame.locals import *
import math
import mazeBank
import numpy

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


def invert(self):
    for index, row in enumerate(self.maze):
        self.maze[index] = row[::-1]

# Check if a cell is a wall or the goal (1 = wall, 2 = goal, 0 = path)
def checkCell(maze, loc_x, loc_y):
    N = maze.info.width
    M = maze.info.height
    # If the cell in the maze array is a 1, the cell is a wall
    if loc_x not in xrange(M) or loc_y not in xrange(N):
        return 1
    elif maze[loc_y][loc_x] == "1":
        return 1
    # If the cell in the maze array is a 2, the cell is the starting position
    elif maze[loc_y][loc_x] == "2":
        return 2
    # If the cell in the maze array is a 2, the cell is the starting position
    elif maze[loc_y][loc_x] == "3":
        return 3
    else:
        return 0

def draw(self, display_surf):
    # Iterate over maze
    for by in range(self.N):
        for bx in range(self.M):
            # If an element is a '1', color it as a wall
            if self.maze[by][bx] == "1":
                pygame.draw.rect(display_surf, PURPLE,
                                 (bx * BLOCKSIZE_X, by * BLOCKSIZE_Y, BLOCKSIZE_X, BLOCKSIZE_Y), 0)
            elif self.maze[by][bx] == "2":
                pygame.draw.rect(display_surf, BLUE,
                                 (bx * BLOCKSIZE_X, by * BLOCKSIZE_Y, BLOCKSIZE_X, BLOCKSIZE_Y), 0)
            elif self.maze[by][bx] == "3":
                pygame.draw.rect(display_surf, RED,
                                 (bx * BLOCKSIZE_X, by * BLOCKSIZE_Y, BLOCKSIZE_X, BLOCKSIZE_Y), 0)

def getStart(maze):
    """
    Get the starting position
    """
    start = maze.data.index(2)
def getGoal(maze):
    """
    Get the goal position
    """
    goal = maze.data.index(3)

def construct_map(maze):
    a = np.array(maze.data)

def index_to_cell(maze,x,y):

    return maze.info.width*x + y

def neighbors_euclidean(maze, loc_x, loc_y):
    neighbors = []
    for x in range(loc_x - 1, loc_x + 2):
        for y in range(loc_y - 1, loc_y + 2):
            if checkCell(maze, x, y) in (0, 2, 3):
                neighbors.append((x, y))

    return neighbors

def neighbors_manhattan(maze,loc_x, loc_y):
    neighbors_in = [(loc_x - 1, loc_y), (loc_x, loc_y + 1), (loc_x + 1, loc_y), (loc_x, loc_y - 1)]
    neighbors_out = []
    for option in neighbors_in:
        if checkCell(maze, option[0], option[1]) in (0, 2, 3):
            neighbors_out.append(option)

    return neighbors_out




if __name__ == "__main__":
theApp = App()
theApp.on_execute()
