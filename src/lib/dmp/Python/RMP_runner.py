import xml.etree.ElementTree as ET
import numpy as np
from math import pi



class RMP_runner(object):
    """docstring for RMP_runner
        Runs the RMP from a training file of motion primatives
    """

    def __init__(self, file, tau=1, dt=0.01):

        self.tau = tau
        self.dt = 0.01
        self.w = None
        self.c = None
        self.h = None
        self.y0 = None
        self.goal = None
        self.y = np.zeros(1)
        self.dy = np.zeros(1)
        self.ddy = np.zeros(1)
        self.x = 1
        self.timesteps = int(2 * pi / self.dt)
        self.readInXML(file)



    @property
    def gen_psi(self):
        """

        :rtype: numpy array
        """
        return np.exp(self.h * (np.cos(self.x - self.c) - 1))

    @property
    def gen_front_term(self):
        """
        Generates the front term on the forcing term.
        For rhythmic DMPs it's non-diminishing, so this
        function is just a placeholder to return 1.
        :return: 1
        """

        if isinstance(self.x, np.ndarray):
            return np.ones(self.x.shape)
        return 1

    def step(self, tau=1.0):
        """Run the DMP system for a single timestep.

        tau float: scales the timestep
                   increase tau to make the system execute faster
        error float: optional system feedback
        """

        alpha_z = 25
        beta_z = 0.25*alpha_z

        # run canonical system
        self.x += 1 * self.tau * self.dt

        # generate basis function activation
        psi = self.gen_psi

        front_term = self.gen_front_term
        # generate the forcing term
        f = (front_term * (np.dot(psi, self.w)) / np.sum(psi))

        # DMP acceleration
        self.ddy = (alpha_z * (beta_z * (self.goal - self.y) - self.dy / tau) + f) * tau
        self.dy += self.ddy * tau * self.dt
        self.y  += self.dy * self.dt

        return self.y, self.dy, self.ddy

    def run(self,tau=1):
        """

        :return:
        """
        y_track = []
        dy_track = []
        ddy_track = []

        for t in xrange(self.timesteps):
            y, dy, ddy = self.step(tau)
            y_track.append(y[0])
            dy_track.append(dy[0])
            ddy_track.append(ddy[0])

        return y_track, dy_track, ddy_track



    def readInXML(self, filename):
        """

        :param filename: name of file
        :return:None
        """

        root = ET.parse(filename).getroot()
        w = []
        h = []
        c = []

        for weight in root.findall("Weights")[0]:
            w.append(float(weight.text))

        for inv_sq in root.findall("inv_sq_var")[0]:
            h.append(float(inv_sq.text))

        for mean in root.findall("gauss_means")[0]:
            c.append(float(mean.text))

        self.w = np.asarray(w)
        self.c = np.asarray(c)
        self.h = np.asarray(h)

        self.goal = float(root.findall("goal")[0].text)
        self.y = float(root.findall("y0")[0].text)




