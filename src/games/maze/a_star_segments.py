#!/usr/bin/env python
"""
    Alexandra Valton

    This node handles the segmentation of A*

"""
import Queue
import maze_helper
import mazeBank
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped


def costmove(current_node, next_node, prev_node):
    if (prev_node is None) or (next_node is None):
        return 1
    elif (next_node[0] == current_node[0] and current_node[0] == prev_node[0]) or (next_node[1] == current_node[1] and current_node[1] == prev_node[1]):
        return 1
    else:
        return 5


def heuristic(a, b):
    (x1, y1) = a
    (x2, y2) = b
    return abs(x1 - x2) + abs(y1 - y2)


if __name__ == '__main__':

    # Get 2D Maze
    maze_2D = mazeBank.getmaze("trainer12")
    row = len(maze_2D)
    col = len(maze_2D[0])
    data = [j for i in maze_2D for j in i]

    # Make it a 1D maze
    my_maze = OccupancyGrid()
    my_maze.data = data
    my_maze.info.width = col
    my_maze.info.height = row

    # Find solution to maze
    start = maze_helper.get_start(my_maze)  # column, row
    goal = maze_helper.get_goal(my_maze)  # column, row
    frontier = Queue.PriorityQueue()
    frontier.put(start)
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0

    while not frontier.empty():
        current = frontier.get()
        for next in maze_helper.neighbors_manhattan(my_maze, current[0], current[1]):
            new_cost = cost_so_far[current] + costmove(current, next, came_from[current])
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(goal, next)
                frontier.put(next, priority)
                came_from[next] = current

    # Reconstruct solution path
    current = goal
    my_path = Path()
    # this_step = PoseStamped()
    step_index = 0
    while current != start:
        my_path.poses.append(PoseStamped())
        my_path.poses[step_index].pose.position.x = current[0]
        my_path.poses[step_index].pose.position.y = current[1]
        my_path.poses[step_index].pose.position.z = 0
        # this_step.header.stamp = rospy.Time.now()
        current = came_from[current]
        step_index += 1
        # print my_path.poses[-1]
    my_path.poses.append(PoseStamped())
    my_path.poses[step_index].pose.position.x = start[0]
    my_path.poses[step_index].pose.position.y = start[1]
    my_path.poses[step_index].pose.position.z = 0
    my_path.poses.reverse()  # optional

    # print my_path
    # Break path into segments
    segment_i = 0
    segments = [[]]
    # print my_path.poses
    for index, step in enumerate(my_path.poses):
        if index == 0:
            prev = step
        else:
            prev = my_path.poses[index - 1]

        try:
            next = my_path.poses[index + 1]
        except IndexError:
            next = step

        # print("Prev: (%d, %d), Step: (%d, %d), Next: (%d, %d)") \
        #      % (prev.pose.position.x, prev.pose.position.y, step.pose.position.x, step.pose.position.y,
        #         next.pose.position.x, next.pose.position.y)

        if (prev is None) or (next is None):
            print("end-bit")
            segments[segment_i].append(step.pose)
        elif not (not (
                next.pose.position.x == step.pose.position.x and step.pose.position.x == prev.pose.position.x) and not step.pose.position.y == next.pose.position.y or not (
                step.pose.position.y == prev.pose.position.y)):
            print("straight")
            segments[segment_i].append(step)
        else:
            print("turn")
            if segments[segment_i]:
                segments.append([])
                segment_i += 1
            segments[segment_i].append(prev)
            segments[segment_i].append(step)
            segments[segment_i].append(next)
            segments.append([])
            segment_i += 1

    print len(segments)
