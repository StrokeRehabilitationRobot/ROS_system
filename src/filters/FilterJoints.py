#!/usr/bin/env python
"""
    Nathaniel Goldfarb

    This node filters the velocity of the joints
"""
import sys
import rospy
from sensor_msgs.msg import JointState
import MeanFilter


# TODO need to ensure both arms are being filtered

class FilterJoints:

    def __init__(self):
        """
            This node subscribes to the unfitlered joints states and using a mean filter to
            smooth the velocity. It then publishes the smoothed joint state.
        """
        rospy.init_node("filter_joints")
        rospy.Subscriber('unfilter_joint_states', JointState, self.filter_callback)
        self.robot_state = rospy.Publisher('joint_states', JointState, queue_size=1, latch=True)
        self.velocity = [MeanFilter.MeanFilter(50), MeanFilter.MeanFilter(50), MeanFilter.MeanFilter(50)]
        self.position = [MeanFilter.MeanFilter(50), MeanFilter.MeanFilter(50), MeanFilter.MeanFilter(50)]

    def filter_callback(self, msg):
        """
        takes in a unfiltered joint state and publishes the smoothed velocity
        :param msg: unfiltered joint state
        :type msg: JointsState()
        :return: None
        """
        updated_vel = []
        updated_pose = []
        for vel, joint in enumerate(self.velocity):
            updated_vel.append(joint.update(msg.velocity[vel]))
            updated_pose.append(joint.update(msg.position[vel]))

        msg.velocity = updated_vel
        self.robot_state.publish(msg)


if __name__ == '__main__':
    FilterJoints()
    while not rospy.is_shutdown():
        rospy.spin()
