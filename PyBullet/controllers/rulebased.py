<<<<<<< Updated upstream:pybullet/controllers/rulebased.py
import random
from numpy import array


class RuleBasedController:

    def __init__(self, env, verbose=False):

        self.env = env
        self.verbose = verbose

        # behavioral parameters
        self.threshold = 0.7
        self.speed = 0.5
=======

class RuleBasedController:

    def __init__(self, env, laser_range=1, speed=0.5, threshold=0.5, verbose=False):
        self._env = env
        self._verbose = verbose
        self._threshold = threshold
        self._laser_range = laser_range
        self.speed = speed
>>>>>>> Stashed changes:PyBullet/controllers/rulebased.py

    def get_command(self):

        # get lasers data
<<<<<<< Updated upstream:pybullet/controllers/rulebased.py
        laserRanges = self.env.get_laserranges()
        laserRanges = [1 - i for i in laserRanges]

        if sum(laserRanges[:5]) > self.threshold:
            if self.verbose:
                print("WALL L", sum(laserRanges[:5]), sum(laserRanges[5:]))
            return [-self.speed, self.speed]

        elif sum(laserRanges[5:]) > self.threshold:
            if self.verbose:
                print("WALL R", sum(laserRanges[:5]), sum(laserRanges[5:]))
            return [self.speed, -self.speed]

        else:
            if self.verbose:
                print("NO WALL ", sum(laserRanges[:5]), sum(laserRanges[5:]))
            return [self.speed, self.speed]
=======
        laser_ranges = self._env.get_laserranges()
        # number of lasers to include for the left and right
        n_rays = len(laser_ranges)//2
        # to make no walls => laserrange = 0
        laser_ranges = [self._laser_range - i for i in laser_ranges]

        if sum(laser_ranges[:n_rays]) > self._threshold:
            if self._verbose:
                print("Wall L ", sum(laser_ranges[:5]), sum(laser_ranges[5:]))
            return [-self.speed, self.speed]

        if sum(laser_ranges[-n_rays:]) > self._threshold:
            if self._verbose:
                print("WALL R ", sum(laser_ranges[:5]), sum(laser_ranges[5:]))
            return [self.speed, -self.speed]

        if self._verbose:
            print("NO WALL ", sum(laser_ranges[:5]), sum(laser_ranges[5:]))
        return [self.speed, self.speed]
>>>>>>> Stashed changes:PyBullet/controllers/rulebased.py

    def reset(self):
        self.threshold = 0.7
        self.speed = 0.5
