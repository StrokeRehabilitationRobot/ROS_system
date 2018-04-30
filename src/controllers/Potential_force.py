import math
import numpy as np


class Attractive_Function():

    def __init__(self, zeta, d_threash):

        self.zeta = zeta
        self.d_threash = d_threash

    def get_U(self,robot_pos,goal_pos):
        """

        :param robot_pos: position of the robot
        :param goal_pos: position of the goal
        :return: engery
        """
        dist = math.sqrt( ( robot_pos[0] - goal_pos[0] ) ** 2 + ( robot_pos[1] - goal_pos[1] ) ** 2 )
        U = 0

        if dist <= self.d_threash:
            U = 0.5 * self.zeta * (dist ** 2)
        else:
            U = self.d_threash*self.zeta*dist - 0.5*self.zeta*(self.d_threash ** 2)

        return U

    def get_nabla_U(self, robot_pos, goal_pos):
        """

        :param robot_pos: position of the robot
        :param goal_pos: position of the goal
        :return: engery
        """

        dist = math.sqrt((robot_pos[0] - goal_pos[0]) ** 2 + (robot_pos[1] - goal_pos[1]) ** 2)

        if dist <= self.d_threash:
            U = self.zeta * ( robot_pos - goal_pos )

        else:
            U =  (self.d_threash*self.zeta * ( np.asarray(robot_pos) - np.asarray(goal_pos) ))/(dist)

        return np.insert(U,2,0,axis=0)



class Repulisive_Function():

    def __init__(self, eta, Q_star):

        self.eta = eta
        self.Q_star = Q_star

    def get_U(self,robot_pos,obs_pos):
        """

        :param robot_pos: position of the robot
        :param goal_pos: position of the goal
        :return: engery
        """
        dist = math.sqrt( ( robot_pos[0] - obs_pos[0] ) ** 2 + ( robot_pos[1] - obs_pos[1] ) ** 2 )
        U = 0

        if dist <= self.Q_star:
            U = 0.5 * self.eta * (  (1.0/dist) - (1.0/self.Q_star)  ) ** 2
        else:
            U = 0
        return np.insert(U,2,0,axis=0)


    def get_nabla_U(self, robot_pos, obs_pos):
        """

        :param robot_pos: position of the robot
        :param goal_pos: position of the goal
        :return: engery
        """

        dist = math.sqrt((robot_pos[0] - obs_pos[0])**2 + (robot_pos[1] - obs_pos[1])**2)
        nabla_dist = (  robot_pos - obs_pos )/( dist )

        U = 0

        if dist <= self.Q_star:
            U = self.eta * ((1.0 / self.Q_star) - (1.0 / dist)) * (nabla_dist/(dist*dist))
        else:
            U = np.array([[0],[0]])

        return np.insert(U,2,0,axis=0)



class Velocity_Attractive_Function():

    def __init__(self, alpha_p, alpha_v, m, n):

        self.alpha_p = alpha_p
        self.alpha_v = alpha_v
        self.m = m
        self.n = n


    def get_U(self,robot_pos,goal_pos):
        """

        :param robot_pos: position of the robot
        :param goal_pos: position of the goal
        :return: engery
        """
        dist = math.sqrt( ( robot_pos[0] - goal_pos[0] ) ** 2 + ( robot_pos[1] - goal_pos[1] ) ** 2 )
        U = 0


        if dist <= self.d_threash:
            U = 0.5 * self.zeta * (dist ** 2)
        else:
            U = self.d_threash*self.zeta*dist - 0.5*self.zeta*(self.d_threash ** 2)

        return U

    def get_nabla_U(self, robot, goal):
        """

        :param robot_pos: position of the robot
        :param goal_pos: position of the goal
        :return: engery
        """
        vec_pos = robot.x - goal.x
        dist_pos = math.sqrt((robot.x.x - goal.x.x) ** 2 + (robot.x.y - goal.x.y) ** 2)
        n = vec_pos/dist_pos
        F_p = self.m*self.alpha_p*(dist_pos**(self.m-1))*n

        vec_pos = robot.xd - goal.xd
        dist_pos = math.sqrt((robot.xd.x - goal.xd.x) ** 2 + (robot.xd.y - goal.xd.y) ** 2)
        n = vec_pos / dist_pos
        F_v = self.n*self.alpha_v*(dist_pos**(self.m-1))*n
        F = F_p + F_v

        return np.insert(F,2,0,axis=0)


class Velocity_Attractive_Function():

    def __init__(self, alpha_p, alpha_v, m, n):

        self.alpha_p = alpha_p
        self.alpha_v = alpha_v
        self.m = m
        self.n = n

    def get_nabla_U(self, robot, goal):
        """

        :param robot_pos: position of the robot
        :param goal_pos: position of the goal
        :return: engery
        """
        vec_pos = goal.x - robot.x
        dist_pos = math.sqrt((robot.x.x - goal.x.x) ** 2 + (robot.x.y - goal.x.y) ** 2)
        n = -vec_pos/dist_pos
        F_p = self.m*self.alpha_p*(dist_pos**(self.m-1))*n

        vec_vel = goal.xd - robot.xd
        dist_vel = math.sqrt((robot.xd.x - goal.xd.x) ** 2 + (robot.xd.y - goal.xd.y) ** 2)
        nv = -vec_vel / (dist_vel+0.001)
        F_v = self.n*self.alpha_v*(dist_pos**(self.n-1))*nv
        F = F_p + F_v

        return np.insert(F,2,0,axis=0)


class Velocity_Repulsive_Function():

    def __init__(self, eta, a_max, pho_0):
        self.eta = eta
        self.a_max = a_max
        self.pho_0 = pho_0

    def get_nabla_U(self, robot, obs):
        """

        :param robot_pos: position of the robot
        :param goal_pos: position of the goal
        :return: engery
        """

        dist = math.sqrt((robot.x.x - obs.x.x)**2 + (robot.x.y - obs.x.y)**2)
        vec = obs.x - robot.x
        n = -vec/dist

        vel = robot.xd - obs.xd
        vel_norm = math.sqrt( vel[0]**2 + vel[1]**2 )

        v_ro = vel[0]*n[0] + vel[1]*n[1]
        v_ro_norm = (robot.xd - obs.xd) - v_ro*n

        pho_m = (v_ro**2)/(2*self.a_max)
        pho_s = dist

        F1 = (-self.eta /( pho_s - pho_m  )**2) * ( 1 + v_ro/self.a_max ) * n
        F2 = ((self.eta * v_ro * v_ro_norm  ) / ( pho_s*self.a_max * ( pho_s - pho_m  )**2 ) )

        if 0 < pho_s - pho_m < self.pho_0 and v_ro > 0:
            F = F1 + F2
            F = np.insert(F,2,0,axis=0)
        else:
            F = np.array([0, 0, 0])


        return F


class DMP_Potential_Function():

    def __init__(self,gamma,beta):

        self.gamma = gamma
        self.beta = beta
        self.R_halfpi = np.array([[np.cos(np.pi / 2.0), -np.sin(np.pi / 2.0)],
                                  [np.sin(np.pi / 2.0), np.cos(np.pi / 2.0)]])

    def static_force(self,cursor,obstical,goal):
        p = np.zeros(2)
        # based on (Hoffmann, 2009)

        y = cursor[0:2]
        dy = cursor[3:5]
        # if we're moving
        if np.linalg.norm(dy) > 1e-5:

            # get the angle we're heading in

            phi_dy = -np.arctan2(dy[1][0], dy[0][0])
            R_dy   =  np.array([[np.cos(phi_dy), -np.sin(phi_dy)],
                                [np.sin(phi_dy), np.cos(phi_dy)]])
            # calculate vector to object relative to body

            obj_vec = obstical - y

            # rotate it by the direction we're going
            obj_vec = np.dot(R_dy, obj_vec)
            # calculate the angle of obj relative to the direction we're going
            phi = np.arctan2(obj_vec[1][0], obj_vec[0][0])

            dphi = self.gamma * phi * np.exp(-self.beta * abs(phi))
            R = np.dot(self.R_halfpi, np.outer(obstical - y, dy))
            p = -np.nan_to_num(np.dot(R, dy) * dphi)

            # check to see if the distance to the obstacle is further than
            # the distance to the target, if it is, ignore the obstacle
            if np.linalg.norm(obj_vec) > np.linalg.norm(goal - y) or np.linalg.norm(obj_vec) > 80  :
                p = np.array([[0],[0]])

        return np.insert(p,2,0,axis=0)


    def velocity_force(self,cursor,obstical,goal):


        # based on (Hoffmann, 2009)
        p = np.array([[0], [0]])
        y = cursor[0:2]
        dy = cursor[3:5]
        do = np.array([[obstical.xd.x], [obstical.xd.y]])
        o = np.array([[obstical.x.x], [obstical.x.y]])
        # if we're moving
        if np.linalg.norm(dy) > 1e-5:

            # get the angle we're heading in

            phi_dy = -np.arctan2(dy[1][0] - do[0], dy[0][0] - do[1])[0]
            R_dy = np.array([[np.cos(phi_dy), -np.sin(phi_dy)],
                             [np.sin(phi_dy), np.cos(phi_dy)]])
            # calculate vector to object relative to body

            obj_vec = o - y
            # rotate it by the direction we're going
            obj_vec = np.dot(R_dy, obj_vec)
            # calculate the angle of obj relative to the direction we're going
            phi = np.arctan2(obj_vec[1][0], obj_vec[0][0])

            dphi = self.gamma * phi * np.exp(-self.beta * abs(phi))
            R = np.dot(self.R_halfpi, np.outer(o - y, dy))
            p = -np.nan_to_num(np.dot(R, dy) * dphi)

            # check to see if the distance to the obstacle is further than
            # the distance to the target, if it is, ignore the obstacle
            if np.linalg.norm(obj_vec) > np.linalg.norm(goal - y) or np.linalg.norm(obj_vec) > 80:
                p = np.array([[0], [0]])

        return np.insert(p, 2, 0, axis=0)
