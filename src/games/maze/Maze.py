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
    def draw(self, display_surf):
        #pygame.draw.rect(display_surf, BLACK,
        #                (self.prev_x, self.prev_y, PLAYERSIZE_X, PLAYERSIZE_Y), 0)
        pygame.draw.rect(display_surf, GREEN,
                         (self.x, self.y, PLAYERSIZE_X, PLAYERSIZE_Y), 0)


class Maze:
    def __init__(self, maze_name = "maze1"):
        #self.M = 15  # number of columns
        #self.N = 12  # number of rows
        #self.invert = True
        ## SELECT MAZE
        self.maze = mazeBank.getmaze(maze_name)
        self.N = len(self.maze) # number of rows
        self.M = len(self.maze[0]) # number of columns

        #####################

    def invert(self):
        for index, row in enumerate(self.maze):
            self.maze[index] = row[::-1]

    # Check if a cell is a wall or the goal (1 = wall, 2 = goal, 0 = path)
    def checkCell(self, loc_x, loc_y):
        # If the cell in the maze array is a 1, the cell is a wall
        if loc_x not in range(0, self.M) or loc_y not in range(0, self.N):
            return 1
        elif self.maze[loc_y][loc_x] == "1":
            return 1
        # If the cell in the maze array is a 2, the cell is the starting position
        elif self.maze[loc_y][loc_x] == "2":
            return 2
        # If the cell in the maze array is a 2, the cell is the starting position
        elif self.maze[loc_y][loc_x] == "3":
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

    def getendpoints(self):
        # Iterate over maze array
        start_loc = (0, 0)
        goal_loc = start_loc
        for by in range(self.N):
            for bx in range(self.M):
                # If an element is a '1', color it as a wall
                if self.maze[by][bx] == "2":
                    start_loc = (bx, by)
                if self.maze[by][bx] == "3":
                    goal_loc = (bx, by)
        return start_loc, goal_loc

    def neighbors_euclidean(self, loc_x, loc_y):
        neighbors = []
        for x in range(loc_x - 1, loc_x + 2):
            for y in range(loc_y - 1, loc_y + 2):
                if self.checkCell(x, y) in (0, 2, 3):
                    neighbors.append((x, y))

        return neighbors

    def neighbors_manhattan(self, loc_x, loc_y):
        neighbors_in = [(loc_x - 1, loc_y), (loc_x, loc_y + 1), (loc_x + 1, loc_y), (loc_x, loc_y - 1)]
        neighbors_out = []
        for option in neighbors_in:
            if self.checkCell(option[0], option[1]) in (0, 2, 3):
                neighbors_out.append(option)

        return neighbors_out


class App:
    windowWidth = 800
    windowHeight = 600
    player = 0

    def __init__(self, maze_name="maze1"):
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


if __name__ == "__main__":
    theApp = App()
    theApp.on_execute()
