from collections import deque
import numpy as np

class MeanFilter():

    def __init__(self,size):

         self.sample_window = deque([], size)

    def update(self,new_sample):
        self.sample_window.append(new_sample)
        return np.sum(self.sample_window, 0)/len(self.sample_window)
