#PONG pygame


import copy

import time
import math
import random
import pygame, sys
import numpy as np
from pygame.locals import *

pygame.init()
fps = pygame.time.Clock()

#colors
WHITE = (255,255,255)
RED = (255,0,0)
GREEN = (0,255,0)
BLACK = (0,0,0)

#globals
WIDTH = 600*2
HEIGHT = 400*2
BALL_RADIUS = 20
PAD_WIDTH = 8
PAD_HEIGHT = 80
HALF_PAD_WIDTH = PAD_WIDTH / 2
HALF_PAD_HEIGHT = PAD_HEIGHT / 2


class pong():

    def __init__(self):
        self._ball_pos = [0,0]
        self._ball_vel = [0,0]
        self._paddle1_vel = 1
        self._paddle2_vel = 1
        self._l_score = 0
        self._r_score = 0
        self._flag = 0
        self._start_timer = 0
        self._vib = False

        self._window = pygame.display.set_mode((WIDTH, HEIGHT), 0, 32)
        pygame.display.set_caption('Hello World')

        self._paddle1_pos = [HALF_PAD_WIDTH - 1, HEIGHT / 2]
        self._paddle2_pos = [WIDTH + 1 - HALF_PAD_WIDTH, HEIGHT / 2]

        canvas = self._window
        canvas.fill(BLACK)
        pygame.draw.line(canvas, WHITE, [WIDTH / 2, 0], [WIDTH / 2, HEIGHT], 1)
        pygame.draw.line(canvas, WHITE, [PAD_WIDTH, 0], [PAD_WIDTH, HEIGHT], 1)
        pygame.draw.line(canvas, WHITE, [WIDTH - PAD_WIDTH, 0], [WIDTH - PAD_WIDTH, HEIGHT], 1)
        pygame.draw.circle(canvas, WHITE, [WIDTH // 2, HEIGHT // 2], 70, 1)

        if random.randrange(0, 2) == 0:
            self.ball_init(True)
        else:
            self.ball_init(False)


        #canvas declaration
#window = pygame.display.set_mode((WIDTH, HEIGHT), 0, 32)
#pygame.display.set_caption('Hello World')

# helper function that spawns a ball, returns a position vector and a velocity vector
# if right is True, spawn to the right, else spawn to the left
    def ball_init(self,right):
        self._ball_pos = [WIDTH/2,HEIGHT/2]
        horz = random.randrange(2,4)
        vert = random.randrange(1,3)

        if right == False:
            horz = - horz

        self._ball_vel = [horz,-vert]


#draw function of canvas
    def draw(self,pos):


        # turn off vibrator
        self._vib = False


        # draw the background
        canvas = self._window
        canvas.fill(BLACK)
        pygame.draw.line(canvas, WHITE, [WIDTH / 2, 0],[WIDTH / 2, HEIGHT], 1)
        pygame.draw.line(canvas, WHITE, [PAD_WIDTH, 0],[PAD_WIDTH, HEIGHT], 1)
        pygame.draw.line(canvas, WHITE, [WIDTH - PAD_WIDTH, 0],[WIDTH - PAD_WIDTH, HEIGHT], 1)
        pygame.draw.circle(canvas, WHITE, [WIDTH//2, HEIGHT//2], 70, 1)

        #save old ball position
        old_ball_pose =  copy.copy(self._ball_pos)

        #update ball
        self._ball_pos[0] += int(self._ball_vel[0])
        self._ball_pos[1] += int(self._ball_vel[1])


        #update the padels location
        self.user_paddle_update(pos)
        self.computer_paddle_update(old_ball_pose)



        #draw paddles and ball
        pygame.draw.circle(canvas, RED, self._ball_pos, 20, 0)
        pygame.draw.polygon(canvas, GREEN, [[self._paddle1_pos[0] - HALF_PAD_WIDTH,   \
                                             self._paddle1_pos[1] - HALF_PAD_HEIGHT], \
                                            [self._paddle1_pos[0] - HALF_PAD_WIDTH,   \
                                             self._paddle1_pos[1] + HALF_PAD_HEIGHT], \
                                            [self._paddle1_pos[0] + HALF_PAD_WIDTH,   \
                                             self._paddle1_pos[1] + HALF_PAD_HEIGHT], \
                                            [self._paddle1_pos[0] + HALF_PAD_WIDTH,   \
                                             self._paddle1_pos[1] - HALF_PAD_HEIGHT]], 0)

        pygame.draw.polygon(canvas, GREEN, [[self._paddle2_pos[0] - HALF_PAD_WIDTH,   \
                                             self._paddle2_pos[1] - HALF_PAD_HEIGHT], \
                                            [self._paddle2_pos[0] - HALF_PAD_WIDTH,   \
                                             self._paddle2_pos[1] + HALF_PAD_HEIGHT], \
                                            [self._paddle2_pos[0] + HALF_PAD_WIDTH,   \
                                             self._paddle2_pos[1] + HALF_PAD_HEIGHT], \
                                            [self._paddle2_pos[0] + HALF_PAD_WIDTH,   \
                                             self._paddle2_pos[1] - HALF_PAD_HEIGHT]], 0)



        #ball collision check on top and bottom walls
        if int(self._ball_pos[1]) <= BALL_RADIUS:
            self._ball_vel[1] = - self._ball_vel[1]
        if int(self._ball_pos[1]) >= HEIGHT + 1 - BALL_RADIUS:
            self._ball_vel[1] = -self._ball_vel[1]

        #ball collison check on gutters or paddles

        if int(self._ball_pos[0]) <= BALL_RADIUS + PAD_WIDTH and \
               int(self._ball_pos[1]) in range(self._paddle1_pos[1] - HALF_PAD_HEIGHT, self._paddle1_pos[1] + HALF_PAD_HEIGHT,1):

            self._ball_vel[0] = -self._ball_vel[0]
            self._ball_vel[0] *= 1.1
            self._ball_vel[1] *= 1.1
            if self._ball_pos[0] < WIDTH/2:
                self._vib = True

        elif int(self._ball_pos[0]) <= BALL_RADIUS + PAD_WIDTH:
            self._r_score += 1
            self.ball_init(True)

        if int(self._ball_pos[0]) >= WIDTH + 1 - BALL_RADIUS - PAD_WIDTH and \
           int(self._ball_pos[1]) in range(self._paddle2_pos[1] - HALF_PAD_HEIGHT,self._paddle2_pos[1] + HALF_PAD_HEIGHT,1):

            self._ball_vel[0] = -self._ball_vel[0]
            self._ball_vel[0] *= 1.1
            self._ball_vel[1] *= 1.1

        elif int(self._ball_pos[0]) >= WIDTH + 1 - BALL_RADIUS - PAD_WIDTH:
            self._l_score += 1
            self.ball_init(False)


        #update scores
        myfont1 = pygame.font.SysFont("Comic Sans MS", 20)
        label1 = myfont1.render("Score "+str(self._l_score), 1, (255,255,0))
        canvas.blit(label1, (50,20))

        myfont2 = pygame.font.SysFont("Comic Sans MS", 20)
        label2 = myfont2.render("Score "+str(self._r_score), 1, (255,255,0))
        canvas.blit(label2, (470, 20))


    def user_paddle_update(self,pos):
        """
        computs the location of the human player padel
        :param pos:
        :return:
        """
        pos = -pos
        # remap the the range from arm space to game space
        OldRange = (0.35 - (-0.35))
        NewRange = ( (HEIGHT-HALF_PAD_HEIGHT) - HALF_PAD_HEIGHT)
        NewValue = int((((pos - (-0.35)) * NewRange) / OldRange) + HALF_PAD_HEIGHT)


        self._paddle1_pos[1] = np.clip(NewValue, HALF_PAD_HEIGHT, HEIGHT - HALF_PAD_HEIGHT)


    def computer_paddle_update(self,old_pose):
        """
        computer the location of the computer player
        :param old_pose: last poisiton of the ball
        :return:
        """
        # Figure out where the ball is heading
        slope =   (old_pose[1] - self._ball_pos[1])/ (old_pose[0] - self._ball_pos[0])

        b = self._ball_pos[1] - slope*self._ball_pos[0]

        if self._ball_pos < 0.5*WIDTH:

            if self._paddle2_pos + HALF_PAD_HEIGHT < 0.5*HEIGHT:
                self._paddle2_pos[1] -= self._paddle2_vel
            elif self._paddle2_pos + HALF_PAD_HEIGHT > 0.5*HEIGHT:
                self._paddle2_pos[1] += self._paddle2_vel


        intersection = slope*(WIDTH) + b
        if self._paddle2_pos[1] < intersection:
            self._paddle2_pos[1] += self._paddle2_vel
        elif self._paddle2_pos[1] > intersection:
            self._paddle2_pos[1] -= self._paddle2_vel


        self._paddle2_pos[1] = np.clip(self._paddle2_pos[1], HALF_PAD_HEIGHT, HEIGHT - HALF_PAD_HEIGHT)

    def update(self,pos):
        """

        :param pos: position of the controller to move the paddle
        :return:
        """

        self.draw(pos)

        pygame.display.update()
        
        return
        fps.tick(20)

        boom = self._vib
        #print boom
        # logic to vibrate the handle.
        # it wil be on for 1 sec after you score
        if boom and not self._flag:
            self._flag = 1
            self._start_timer = time.clock()
            return 1

        elif self._flag:
            now = time.clock()
            if math.fabs(now - self._start_timer) > 1:
                self._flag = 0
                return 0
            else:
                return 1
        else:
            return 0
