#!/usr/bin/env python

import rospy
from strokeRehabSystem.srv import *
from strokeRehabSystem.msg import *
from std_msgs.msg import Bool
import csv



def state_callback(msg):
    global at_goal
    global at_start
    global states



    if at_start and not at_goal:
        print "record"
        state = [msg.pose.position.x, msg.pose.position.y,msg.pose.position.z,\
                  msg.vel.linear.x,msg.vel.linear.y,msg.vel.linear.z, \
                  msg.accel.linear.x,msg.accel.linear.y,msg.accel.linear.z]

        states.append(state)


def goal_callback(msg):

    global at_goal
    global states

    at_goal = msg.data == True
    print "saving"
    with open("down2.csv", "w") as file:
        writer1 = csv.writer(file)
        writer1.writerow(['x', 'y','z', 'xd', 'yd', 'zd', 'xdd', 'ydd', 'zdd'])
        for row in states:
            print row
            writer1.writerow(row)

def start_callback(msg):

    global at_start
    print "Start"
    at_start = msg.data == True


if __name__ == '__main__':

    global at_goal
    global at_start
    global states


    at_goal = False
    at_start = False
    states = []

    rospy.init_node("recorder")
    rospy.Subscriber("/Player_State", PlayerState, state_callback)
    rospy.Subscriber("/at_goal", Bool, goal_callback )
    rospy.Subscriber("/at_start", Bool,start_callback)

    rospy.spin()