#!/usr/bin/env python

import sys
import pygame
from pygame.locals import *
import numpy as np
from nav_msgs.msg import OccupancyGrid,Path
from strokeRehabSystem.srv import ReturnJointStates
from geometry_msgs.msg import Pose,Point, WrenchStamped
from std_msgs.msg import Bool
import rospy
import time
import tf
import tools.joint_states_listener
import tools.helper
import controllers.HapticController
from operator import sub
import time
rospy.init_node('pytalker')
odom_list = tf.TransformListener()

count = 0
while count < 10:
    t0 = time.time()
    p = tools.helper.call_return_joint_states()
    print time.time() - t0

    t0 = time.time()
    odom_list.waitForTransform('base_link', 'master_EE', rospy.Time(0),rospy.Duration(0.1))
    (position, _ ) = odom_list.lookupTransform('base_link', 'master_EE', rospy.Time(0))
    print time.time() - t0
    count= count+1
    rospy.sleep(2)
