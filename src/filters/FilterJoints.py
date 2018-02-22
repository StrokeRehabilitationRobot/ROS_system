#!/usr/bin/env python

import sys
import rospy
from sensor_msgs.msg import JointState
import MeanFilter


class FilterJoints():

    def __init__(self):
        rospy.init_node("filter_joints")
        rospy.Subscriber('unfilter_joint_states', JointState, self.filter)
        self.robot_state = rospy.Publisher('joint_states', JointState, queue_size=1,latch=True)
        self.velocity = [MeanFilter.MeanFilter(50),MeanFilter.MeanFilter(50),MeanFilter.MeanFilter(50)]

    def filter(self,msg):
        updated_vel = []
        for  vel, joint in enumerate(self.velocity):
            updated_vel.append(joint.update(msg.velocity[vel]))

        msg.velocity = updated_vel
        self.robot_state.publish(msg)


if __name__ == '__main__':
    filter = FilterJoints()
    while not rospy.is_shutdown():
        rospy.spin()
