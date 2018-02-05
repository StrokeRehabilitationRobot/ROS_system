#!/usr/bin/env python
import rospy
import maze_helper
import mazeBank
from geometry_msgs.msg import Path
from nav_msgs.msg import OccupancyGrid

def make_maze(maze):
    maze_pub = rospy.Publisher('tmaze', OccupancyGrid, queue_size=1)

    row = len(maze)
    col = len(maze[0])
    data = [j for i in maze for j in i]
    my_maze = OccupancyGrid()

    my_maze.data = data
    my_maze.info.width = row
    my_maze.info.height = col
    my_maze.header.stamp = rospy.Time.now()





if __name__ == '__main__':
    rospy.init_node("maze")
    maze_name = sys[0]
    my_maze = mazeBank(make_name)
    make_maze(maze)

    while not rospy.is_shutdown():
        pass
