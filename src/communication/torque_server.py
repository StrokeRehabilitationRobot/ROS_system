import rospy
from geometery_msgs.msg import WrenchStamped
import tools.helper
import tools.dynamics
import numpy as np


def make_torque_callback(force):

    (position, velocity, effort) = tools.helper.call_return_joint_states()

    if force.header.frame_id == "slave":
        board = 0
    else:
        board = 1

    J = tools.dynamics.get_J_tranpose(position)
    tau = np.array(J).dot(np.array(force).reshape(3, 1))



if __name__ == '__main__':

    rospy.init_node('torque_server')
    rospy.Subscriber("chatter", WrenchStamped, make_torque_callback)
