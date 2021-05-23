import random
from numpy import array


class BraitenbergController:

<<<<<<< Updated upstream:pybullet/controllers/braitenberg.py
    def __init__(self, env, verbose=False):

        self.env = env
        self.verbose = verbose

        # behavioral parameters
        self.reactivity = 0.02
        self.speed = 1
=======
    def __init__(self, env, laser_range=1, speed=1, reactivity=1, verbose=False):
        self._env = env
        self._verbose = verbose
        self._reactivity = reactivity
        self._speed = speed
        self._laser_range = laser_range
>>>>>>> Stashed changes:PyBullet/controllers/braitenberg.py

    def get_command(self):

        # get lasers data
<<<<<<< Updated upstream:pybullet/controllers/braitenberg.py
        laserRanges = self.env.get_laserranges()

        laserRanges = [4 - i for i in laserRanges]
        # print(laserRanges)
        Sr = sum(laserRanges[:5])
        Sl = sum(laserRanges[5:])
=======
        laser_ranges = self._env.get_laserranges()
        # number of lasers to include for the left and right
        n_rays = len(laser_ranges)//2
        # to make no walls => laserrange = 0
        laser_ranges = [self._laser_range - i for i in laser_ranges]
        sl = sum(laser_ranges[:n_rays])
        sr = sum(laser_ranges[-n_rays:])

        right = self._speed*(1+self._reactivity*(sl-sr))
        left = self._speed*(1+self._reactivity*(sr-sl))
>>>>>>> Stashed changes:PyBullet/controllers/braitenberg.py

        left = self.speed*(1+self.reactivity*(Sl-Sr))
        right = self.speed*(1+self.reactivity*(Sr-Sl))

        if self.verbose:
            print("Sr:", Sr, "Sl:", Sl, "left:",
                  left, 'right:', right)
        return [left, right]

    def reset(self):
        self.reactivity = 0.02
        self.speed = 1
