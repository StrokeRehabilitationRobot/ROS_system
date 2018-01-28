#!/usr/bin/env python


from strokeRehabSystem.msg import *
from sensor_msgs.msg import JointState
import tools.helper
import tools.dynamics
import rospy
import math
import pong
game = pong.pong()
import time
def callback(msg):
        time0 = time.clock()
        pos0, pos1, pos2 = tools.dynamics.fk(msg.position)
        #print pos1
        game.update(pos2[1])
        print time0 - time.clock()

if __name__ == "__main__":
        rospy.init_node('pong', anonymous=True)
        #game =
        rospy.Subscriber("/joint_states", JointState, callback)
        rospy.spin()
