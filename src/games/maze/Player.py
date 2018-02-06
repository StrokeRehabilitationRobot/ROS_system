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


class Player:
    # Player position is in pixels
    # Initialize player to block in second row, second column
    x = BLOCKSIZE_X + 1
    y = BLOCKSIZE_Y + 1
    prev_x = x
    prev_y = y
    ## CHANGE PLAYER SPEED HERE
    speed = 2
    ########

    # Move right relative to current position, using speed modified by magnitude (mag)
    def moveRight(self, mag):
        self.prev_x = self.x
        self.x = self.x + (self.speed*mag)

    # Move left relative to current position, using speed modified by magnitude (mag)
    def moveLeft(self, mag):
        self.prev_x = self.x
        self.x = self.x - (self.speed*mag)

    # Move up relative to current position, using speed modified by magnitude (mag)
    def moveUp(self, mag):
        self.prev_y = self.y
        self.y = self.y - (self.speed*mag)

    # Move down relative to current position, using speed modified by magnitude (mag)
    def moveDown(self, mag):
        self.prev_y = self.y
        self.y = self.y + (self.speed*mag)

    # Move to a position, ideally representing the end effector position of the robot
    def goTo(self, loc_x, loc_y):
        self.x = loc_x
        self.y = loc_y

    # Draw the player inside the maze
