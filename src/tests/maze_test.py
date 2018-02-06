#!/usr/bin/env python
from sensor_msgs.msg import JointState
import tools.helper

from src.games.maze import Maze
import games.maze



maze = Maze.App("trainer3")
arm_translation_x = 0.25
arm_scale_x = 0.35
arm_translation_y = 0.15
arm_scale_y = 0.35


def callback_maze(msg):

    pos0, pos1, pos2 = tools.dynamics.fk(msg.position)

    op_space = [(arm_translation_x - pos2[1]) / arm_scale_x,
                (pos2[0] - arm_translation_y) / arm_scale_y]
    maze.arm_pos(op_space[0], op_space[1])
    maze.on_loop()
    maze.on_render_full()
    maze_Done = maze.is_start()



def maze_server():
    rospy.init_node('maze')
    rospy.Subscriber("/joint_states", JointState, callback_maze)
    #udp = UDP.UDP(9876)
    rospy.spin()


if __name__ == '__main__':
    main()

robot = Robot.Robot("arm1",id=0)
#ploter = PlotArm.PlotArm()

VEL_CONTROL = 48
VEL_TARGET = 42
PID_CONTROL = 37
PID_CONFIG = 65
REC_LENGTH = 20


maze = Maze.App("trainer3")

packet = 15*[0.0]

pidConstants = [0.001, 0.0005, .01, .001, .0005, .01, 0.002, 0.0004, 0.01, 0, 0, 0, 0, 0, 0];
maze.on_init()
maze_Done = False

while(time.time() < end_time) and (maze_Done == False):
    packet[0] = helper.angle_to_encoder(0)
    packet[3] = helper.angle_to_encoder(0.25 * math.pi)
    packet[6] = helper.angle_to_encoder(0)
    upstream = udp.send_packet(0, 37, packet)
    robot.update(upstream)
    end_effector_position = Dynamics.fk(robot)
    op_space = [(arm_translation_x - end_effector_position[2][1]) / arm_scale_x,
                (end_effector_position[2][0] - arm_translation_y) / arm_scale_y]
    maze.arm_pos(op_space[0], op_space[1])
    maze.on_loop()
    maze.on_render_full()
    maze_Done = maze.is_start()

    # print robot.q
start_time = time.time()
end_time = start_time + REC_LENGTH
print("Starting at " + str(start_time) + " and recording for " + str(REC_LENGTH) + " seconds")
maze_Done = False
while(time.time() < end_time) and (maze_Done == False):

    #u = controller.getTorque(robot)
    packet[0] = helper.angle_to_encoder(0)
    packet[3] = helper.angle_to_encoder(0.25*math.pi)
    packet[6] = helper.angle_to_encoder(0)
    upstream = udp.send_packet(0, 37, packet)
    robot.update(upstream)
    end_effector_position = Dynamics.fk(robot)
    op_space = [(arm_translation_x - end_effector_position[2][1])/arm_scale_x,
                (end_effector_position[2][0] - arm_translation_y)/arm_scale_y]
    recorder.writerow([time.time() - start_time, robot.q[0], robot.q[1], robot.q[2]])
    #ploter.update(*Dynamics.fk(robot))
    maze.arm_pos(op_space[0], op_space[1])
    maze.on_loop()
    maze.on_render()
    maze_Done = maze.is_done()

maze.on_cleanup()
print("Recording complete")
