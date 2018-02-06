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

class App():
    windowWidth = 800
    windowHeight = 600
    player = 0

    def __init__(self, maze_name="maze1", node):
        self._running = True
        self._display_surf = None
        self.player = Player()
        self.maze = Maze(maze_name)

    # Initialize game, window, maze
    def on_init(self):
        pygame.init()
        self._display_surf = pygame.display.set_mode((self.windowWidth, self.windowHeight))
        pygame.display.set_caption('Travel from Blue Square to Red Square')
        self._running = True
        self.maze.draw(self._display_surf)
        start_loc, end_loc = self.maze.getendpoints()
        start_loc_pixels_x = (start_loc[0] * BLOCKSIZE_X) + math.floor(abs((BLOCKSIZE_X - PLAYERSIZE_X) * 0.5))
        start_loc_pixels_y = (start_loc[1] * BLOCKSIZE_Y) + math.floor(abs((BLOCKSIZE_Y - PLAYERSIZE_Y) * 0.5))
        self.player.goTo(start_loc_pixels_x, start_loc_pixels_y)
        self.player.draw(self._display_surf)
        pygame.display.update()

    # Not used
    def on_event(self, event):
        if event.type == QUIT:
            self._running = False

    # Empty loop function, called regularly during operation
    def on_loop(self):
        pass

    # Re-render maze (just draws new player position, since player path is preserved)
    def on_render(self):
        #self._display_surf.fill((0, 0, 0))
        #self.maze.draw(self._display_surf)
        self.player.draw(self._display_surf)
        pygame.display.update()

    def on_render_full(self):
        self._display_surf.fill((0, 0, 0))
        self.maze.draw(self._display_surf)
        self.player.draw(self._display_surf)
        pygame.display.update()

    # Exit sequence
    def on_cleanup(self):
        pygame.quit()

    # Execute a movement based on arm movement (velocity mode)
    # Check the coordinates of the end effector (y-value shifted)
    # Move arm at a speed proportional to the distance from the 0 point on the axis
    def arm_move(self, x_sign, y_sign):
        if ((y_sign - Y_CUTOFF) < (0 - THRESHOLD)):
            print('Y value is ' + str(y_sign - Y_CUTOFF) + ' and direction is up')
            self.player.moveUp(2*abs(y_sign - Y_CUTOFF))

        if ((y_sign - Y_CUTOFF) > (0 + THRESHOLD)):
            print('Y value is ' + str(y_sign - Y_CUTOFF) + ' and direction is down')
            self.player.moveDown(2*abs(y_sign - Y_CUTOFF))

        if (x_sign < (0 - THRESHOLD)):
            print('X value is ' + str(x_sign) + ' and direction is right')
            self.player.moveRight(abs(x_sign))

        if (x_sign > (0 + THRESHOLD)):
            print('X value is ' + str(x_sign) + ' and direction is left')
            self.player.moveLeft(abs(x_sign))

    # Execute a movement based on arm position, given as a percent of the operational space, (0,0) in top left corner
    def arm_pos(self, loc_x, loc_y):
        grid = [loc_x*self.maze.M,loc_y*self.maze.N]
        task_space_position = numpy.multiply(grid,[BLOCKSIZE_X, BLOCKSIZE_Y])
        self.player.goTo(task_space_position[0], task_space_position[1])

    def is_done(self):
        start, goal = self.maze.getendpoints()
        goal_pixels = (goal[0] * BLOCKSIZE_X, goal[1] * BLOCKSIZE_Y)
        if abs(self.player.x - goal_pixels[0]) < PLAYERSIZE_X and abs(self.player.y - goal_pixels[1]) < PLAYERSIZE_Y:
            print('Player is at %d, %d, Goal is at %d, %d, and that is close enough') %(self.player.x, self.player.y, goal_pixels[0], goal_pixels[1])
            return True
        else:
            return False

    def is_start(self):
        start, goal = self.maze.getendpoints()
        start_pixels = (start[0] * BLOCKSIZE_X, start[1] * BLOCKSIZE_Y)
        if abs(self.player.x - start_pixels[0]) < PLAYERSIZE_X and abs(
                        self.player.y - start_pixels[1]) < PLAYERSIZE_Y:
            print('Player is at %d, %d, Start is at %d, %d, and that is close enough') % (
            self.player.x, self.player.y, start_pixels[0], start_pixels[1])
            return True
        else:
            return False

    # When app is called from this file, this is the execution loop
    def on_execute(self):
        # Initialize maze
        if self.on_init() == False:
            self._running = False

        # Execution loop - use keys as input, ESC to quit
        while (self._running):
            pygame.event.pump()
            keys = pygame.key.get_pressed()


            if (keys[K_RIGHT]):
                self.player.moveRight(1)

            if (keys[K_LEFT]):
                self.player.moveLeft(1)

            if (keys[K_UP]):
                self.player.moveUp(1)

            if (keys[K_DOWN]):
                self.player.moveDown(1)

            if (keys[K_ESCAPE]):
                self._running = False

            self.on_loop()
            self.on_render()
        self.on_cleanup()
