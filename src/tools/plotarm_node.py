#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseArray
import PlotArm


plotter = None

def callback():pass

if __name__ == "__main__":
	rospy.init_node('listener', anonymous=True)
	rospy.Subscriber("plotter", PoseArray, callback)
	ploter = PlotArm.PlotArm()
	rospy.spin()
		





