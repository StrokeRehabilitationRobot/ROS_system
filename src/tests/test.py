#!/usr/bin/env python
import rospy
class ClassName(object):
    """docstring for ."""
    def __init__(self):
        rospy.init_node('temp')
        rospy.Timer(rospy.Duration(.1), self.timerCallback)

    def timerCallback(self,mag):
        rospy.loginfo("hello_str")

if __name__ == '__main__':
    ClassName()
    while 1:
        pass
