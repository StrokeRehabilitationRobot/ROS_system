#!/usr/bin/env python

import numpy as np
from matplotlib import pyplot as plt
import rospy
from sensor_msgs.msg import JointState
from mpl_toolkits.mplot3d import Axes3D
import dynamics


def plot(msg):

	global figure
	global plt
	global lines
	p_1,p_2,p_3 = dynamics.fk(msg.position)
	xdata = [p_1[0], p_2[0], p_3[0] ]
	ydata = [p_1[1], p_2[1], p_3[1]]
	zdata = [p_1[2], p_2[2], p_3[2]]
	Axes3D.plot(xdata,ydata,zs=zdata)
	plt.draw()


if __name__ == '__main__':


	rospy.init_node("plotter")
	rospy.Subscriber("/joint_states", JointState, plot)
	figure = plt.figure()
	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')
	lines, = ax.plot([], [],[], '-o')
	plt.ion()
	plt.show()
	rospy.spin()

# import rospy
# from geometry_msgs.msg import PoseArray
# from sensor_msgs.msg import JointState
# import PlotArm
# import helper
# import dynamics
#
# plotter = PlotArm.PlotArm()
#
# def callback_plot(joint_state):
# 	joint_location = dynamics.fk(joint_state.position)
# 	print joint_location
# 	plotter.update(*joint_location)
#
# if __name__ == "__main__":
# 	rospy.init_node('plotter', anonymous=True)
# 	rospy.Subscriber("/joint_states", JointState, callback_plot)
# 	rospy.spin()
