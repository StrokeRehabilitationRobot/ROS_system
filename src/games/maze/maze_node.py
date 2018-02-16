#!/usr/bin/env python
import numpy as np
import rospy
import maze_helper
import mazeBank
from std_msgs.msg import Bool
from nav_msgs.msg import OccupancyGrid


maze_pub = rospy.Publisher('gen_maze', OccupancyGrid, queue_size=1, latch=True)



def make_maze(msg):
    maze_names = [ "maze1","trainer12"]
    rand = np.random.randint(0, len(maze_names))

    maze = mazeBank.getmaze(maze_names[rand])

    row = len(maze)
    col = len(maze[0])
    data = [j for i in maze for j in i]

    my_maze = OccupancyGrid()

    my_maze.data = data
    my_maze.info.width = col
    my_maze.info.height = row
    my_maze.header.stamp = rospy.Time.now()
    maze_pub.publish(my_maze)


if __name__ == '__main__':

    rospy.init_node("maze_launch")
    rospy.Subscriber("at_goal", Bool, make_maze)
    make_maze( Bool())

    while not rospy.is_shutdown():
        rospy.spin()
