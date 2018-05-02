"""
    Nathaniel Goldfarb

    Mean filter for joints
"""
from collections import deque
from numpy import sum


class MeanFilter:

    def __init__(self, size):
        """

        :param size: window size
        :type size: int

        """
        self.sample_window = deque([], size)

    def update(self, new_sample):
        """
        take in the new value and appends it to the list then returns the average value
        :param new_sample: new sample to add to the queue
        :return: average value
        """
        self.sample_window.append(new_sample)
        return sum(self.sample_window, 0) / len(self.sample_window)
