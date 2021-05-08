import random
from numpy import array

speed = 0.00425
threshold = 0.7


class RuleBasedController:

    def __init__(self, env, verbose=False):

        self.env = env
        self.verbose = verbose

        # behavioral parameters
        self.threshold = threshold
        self.speed = speed

    def get_command(self):

        # get lasers data
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

    def reset(self):
        self.threshold = threshold
        self.speed = speed
