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
        state = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
                 msg.vel.linear.x, msg.vel.linear.y, msg.vel.linear.z,
                 msg.accel.linear.x, msg.accel.linear.y, msg.accel.linear.z]

        states.append(state)


def goal_callback(msg):
    """
    Stop on goal readh
    :param msg: goal message
    :type msg: Bool()
    :return:
    """
    global at_goal
    global states

    at_goal = True == msg.data
    print "saving"
    # TODO pass in file name
    with open("down2.csv", "w") as FILE:
        writer1 = csv.writer(FILE)
        writer1.writerow(['x', 'y', 'z', 'xd', 'yd', 'zd', 'xdd', 'ydd', 'zdd'])
        for row in states:
            print row
            writer1.writerow(row)


def start_callback(msg):
    """
    callback to start recording
    :param msg: start message
    :type: Bool()
    :return:
    """
    global at_start
    print "Start"
    at_start = True == msg.data


if __name__ == '__main__':
    global at_goal
    global at_start
    global states

    rospy.init_node("recorder")
    rospy.Subscriber("/Player_State", PlayerState, state_callback)
    rospy.Subscriber("/at_goal", Bool, goal_callback)
    rospy.Subscriber("/at_start", Bool, start_callback)

    rospy.spin()
