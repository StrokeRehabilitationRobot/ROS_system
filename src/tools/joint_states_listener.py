#!/usr/bin/env python
#spins off a thread to listen for joint_states messages
#and provides the same information (or subsets of) as a service
import numpy as np
import roslib
#roslib.load_manifest('joint_states_listener')
import rospy
from strokeRehabSystem.srv import *
from sensor_msgs.msg import JointState
import threading
from collections import deque


#holds the latest states obtained from joint_states messages
class LatestJointStates:

    def __init__(self):
        queue_size = 5
        rospy.init_node('joint_states_listener')
        self.lock = threading.Lock()

        self.name = 3*[0]
        self.position = 3*[0]
        self.velocity = 3*[0]
        self.effort = 3*[0]

        self.position_queue = deque([], queue_size)

        self.thread = threading.Thread(target=self.joint_states_listener)
        self.thread.start()

        s = rospy.Service('return_joint_states', ReturnJointStates, self.return_joint_states)


    #thread function: listen for joint_states messages
    def joint_states_listener(self):
        rospy.Subscriber('joint_states', JointState, self.joint_states_callback)
        rospy.spin()


    #callback function: when a joint_states message arrives, save the values
    def joint_states_callback(self, msg):
        self.lock.acquire()
        rospy.loginfo("messages received!\n")
        self.name = msg.name
        self.position_queue.append(msg.position)
        if msg.position == [0,0,0]:
            print("Zero position message recieved")
        self.position = np.sum(self.position_queue, 0)/len(self.position_queue)
        #self.position = msg.position
        self.velocity = msg.velocity
        self.effort= msg.effort
        self.lock.release()


    #returns (found, position, velocity, effort) for the joint joint_name
    #(found is 1 if found, 0 otherwise)
    def return_joint_state(self, joint_name):

        #no messages yet
        if self.name == []:
            rospy.logerr("No robot_state messages received!\n")
            return (0, 0., 0., 0.)

        #return info for this joint
        self.lock.acquire()

        if joint_name in self.name and not len(self.velocity) ==0 :

            index = self.name.index(joint_name)
            position = self.position[index]
            velocity = self.velocity[index]
            effort = self.effort[index]

        #unless it's not found
        else:

            #rospy.logerr("Joint %s not found!", (joint_name,))
            index = self.name.index
            position = self.position
            velocity = self.velocity
            effort = self.effort

            self.lock.release()

            return (0, 0., 0., 0.)
        self.lock.release()

        return (1, position, velocity, effort)


    #server callback: returns arrays of position, velocity, and effort
    #for a list of joints specified by name
    def return_joint_states(self, req):
        joints_found = []
        positions = []
        velocities = []
        efforts = []
        for joint_name in req.name:
            (found, position, velocity, effort) = self.return_joint_state(joint_name)
            joints_found.append(found)
            positions.append(position)
            velocities.append(velocity)
            efforts.append(effort)
        return ReturnJointStatesResponse(joints_found, positions, velocities, efforts)


#run the server
if __name__ == "__main__":

    latestjointstates = LatestJointStates()

    print "joints_states_listener server started, waiting for queries"
    rospy.spin()
