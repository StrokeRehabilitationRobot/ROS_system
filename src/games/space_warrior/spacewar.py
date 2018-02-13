#!/usr/bin/env python

import numpy
import sys
from nav_msgs.msg import OccupancyGrid,Path
from strokeRehabSystem.srv import ReturnJointStates
from geometry_msgs.msg import Pose,Point, WrenchStamped
from std_msgs.msg import Bool
import rospy
import tf
import tools.joint_states_listener
import tools.helper
import pygame
import random
from pygame import *
import rospy
import time
import os
import random
import turtle
# turtle.fd(0)
turtle.speed(0)
turtle.bgcolor("black")
turtle.title("Space warrior for Stroke Rehab")
turtle.bgpic("starfield.gif")
turtle.ht()
turtle.setundobuffer(1)
turtle.tracer(0)# turtle drawing speed

class Sprite(turtle.Turtle):
    def __init__(self,spriteshape,color,startx,starty):
        turtle.Turtle.__init__(self,shape = spriteshape)
        self.speed(0) #speed of animation
        self.penup()
        self.color(color)
        self.fd(0)
        self.goto(startx,starty)
        self.speed = 1

    def move(self):
        self.fd(self.speed)
        if self.xcor() >390:
            self.setx(390)
            self.rt(60)

        if self.xcor() <-390:
            self.setx(-390)
            self.rt(60)

        if self.ycor() >390:
            self.sety(390)
            self.rt(60)

        if self.ycor() <-390:
            self.sety(-390)
            self.rt(60)

    def collision(self,other):
        if (self.xcor() >=(other.xcor()-20)) and \
        (self.xcor() <=(other.xcor()+20)) and \
        (self.ycor() >=(other.ycor()-20)) and \
        (self.ycor() <=(other.ycor()+20)):
            return True
        else:
            return False

class Player(Sprite):
    def __init__(self,spriteshape,color,startx,starty):
        Sprite.__init__(self,spriteshape,color,startx,starty)
        self.shapesize(stretch_wid=1.1,stretch_len = 2.1, outline = None)
        self.speed = 4
        self.lives = 10#change accordingly

    def move(self):
        (position, velocity, effort) = tools.helper.call_return_joint_states()
        EE_y = tools.helper.remap(position[0],-0.6,0.6,0,800 )
        EE_x = tools.helper.remap(position[2],2.1,0.6,0,800 )
        self.sety(EE_x)
        self.setx(EE_y)

    def move_left(self):
        self.lt(45)

    def move_right(self):
        self.rt(45)

    def speed_up(self):
        self.speed+=1

    def speed_down(self):
        self.speed-=1

class Enemy(Sprite):
    def __init__(self,spriteshape,color,startx,starty):
        Sprite.__init__(self,spriteshape,color,startx,starty)
        self.speed = 6
        self.setheading(random.randint(0,360))

class Friend(Sprite):
    def __init__(self,spriteshape,color,startx,starty):
        Sprite.__init__(self,spriteshape,color,startx,starty)
        self.speed = 10
        self.setheading(random.randint(0,360))
    def move(self):
        self.fd(self.speed)
        if self.xcor() >390:
            self.setx(390)
            self.lt(60)

        if self.xcor() <-390:
            self.setx(-390)
            self.lt(60)

        if self.ycor() >390:
            self.sety(390)
            self.lt(60)

        if self.ycor() <-390:
            self.sety(-390)
            self.lt(60)


# class Missile(Sprite):
#     def __init__(self,spriteshape,color,startx,starty):
#         Sprite.__init__(self,spriteshape,color,startx,starty)
#         self.shapesize(stretch_wid = 0.2,stretch_len = 0.4,outline= None)
#         self.speed = 20
#         self.status = "ready"
#         self.goto(-1000,1000)
#
#
#     def fire(self):
#         if self.status == "ready":#self.goto(-1000,1000)
#             self.goto(player.xcor(),player.ycor())
#             self.setheading(player.heading())
#             self.status ="shoot"
#
#     def move(self):
#
#         if self.status == "ready":
#             self.goto(-1000,1000)
#         if self.status == "shoot":
#             self.fd(self.speed)
#
#         if self.xcor()>390 or \
#         self.xcor()<-390 or \
#         self.ycor()>390 or \
#         self.ycor()<-390:
#             self.goto(-1000,1000)
#             self.status = "ready"




class Game():
    def __init__(self):
        self.level = 1
        self.score = 0
        self.state = "playing"
        self.pen = turtle.Turtle()
        self.lives = 100

    def border(self):
        self.pen.speed(0)
        self.pen.color("white")
        self.pen.pensize(3)
        self.pen.penup()
        self.pen.goto(-400,400)
        self.pen.pendown()
        for side in range(4):
            self.pen.fd(800)
            self.pen.rt(90)
        self.pen.penup()
        self.pen.ht()
        self.pen.pendown()

    def status(self):
        self.pen.undo()
        message = "Score: %s" %(self.score)
        self.pen.penup()
        self.pen.goto(-400,410)
        self.pen.write(message,font=("Arial",15,"normal"))



if __name__ == '__main__':

    rospy.init_node("space")
    game = Game()

    game.border()
    game.status()
    #sprite
    player = Player("triangle","yellow",0,0)
    #enemy = Enemy("circle","red",-100,0)
    # missile = Missile("triangle","yellow",0,0)
    #friend = Friend("square","blue",0,0)
    enemies=[]
    for i in range(3):
        enemies.append(Enemy("circle","red",-100,0))
    friends = []
    for i in range(3):
        friends.append(Friend("square","blue",100,0))
    #KEYBOARD
    # turtle.listen()
    # turtle.onkey(player.move_left,"Left")
    # turtle.onkey(player.move_right,"Right")
    # turtle.onkey(player.speed_up,"Up")
    # turtle.onkey(player.speed_down,"Down")
    # turtle.onkey(missile.fire,"space")
    while True:
        turtle.update()
        time.sleep(0.02)
        player.move()
        #enemy.move()
        # missile.move()
        #friend.move()
        #player.move_left()
        for enemy in enemies:
            enemy.move()
            if player.collision(enemy):
                x = random.randint(-350,350)
                y = random.randint(-350,350)
                enemy.goto(x,y)
                game.score +=100
                game.status()

            # if missile.collision(enemy):
            #     x = random.randint(-350,350)
            #     y = random.randint(-350,350)
            #     enemy.goto(x,y)
            #     missile.status ="ready"
            #     game.score+=100
            #     game.status()

        for friend in friends:
            friend.move()
            if player.collision(friend):
                x = random.randint(-350,350)
                y = random.randint(-350,350)
                friend.goto(x,y)
                #missile.status ="ready"
                game.score-=50
                game.status()

    delay = raw_input("press enter")
