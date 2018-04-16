import rospy
from strokeRehabSystem import *
from std_msgs.msg import Bool



def stat_callback(msg):
    global at_goal
    global at_start
    global Xstates
    global Ystates
    global Zstates


    if at_start and not at_goal:

        xstate = [msg.pose.position.x, msg.vel.linear.x,msg.accel.linear.x]
        ystate = [msg.pose.position.y, msg.vel.linear.y, msg.accel.linear.y]
        zstate = [msg.pose.position.z, msg.vel.linear.z, msg.accel.linear.z]

        Xstates.append(xstate)
        Ystates.append(ystate)
        Zstates.append(zstate)


def goal_callback(msg)

    global at_goal

    at_goal = msg.data == True


def start_callback(msg)

    global at_start

    at_start = msg.data == True

if __name__ == '__main__':

    global at_goal
    global at_start
    global Xstates
    global Ystates
    global Zstates

    at_goal = False
    at_start = False
    Xstates = []
    Ystates = []
    Zstates = []

	rospy.init_node("recorder")
	rospy.Subscriber("/player_state", PlayerState, state_callback)
    rospy.Subscriber("/at_goal", Bool, goal_callback )
    rospy.Subscriber("/at_start", Bool,start_callback)

	rospy.spin()