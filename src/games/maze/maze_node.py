#!/usr/bin/env python
import rospy
import maze_helper
import mazeBank

from nav_msgs.msg import OccupancyGrid


maze_pub = rospy.Publisher('gen_maze', OccupancyGrid, queue_size=1)
def make_maze(maze):



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
    my_maze = mazeBank.getmaze("trainer12")
    make_maze(my_maze)

    while not rospy.is_shutdown():
        rospy.spin()

