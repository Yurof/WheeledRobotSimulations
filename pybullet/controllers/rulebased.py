"""
"""


class RuleBasedController:

    def __init__(self, env, laser_range=1, speed=0.5, threshold=0.7, verbose=False):
        self._env = env
        self._verbose = verbose
        self._threshold = threshold
        self._laser_range = laser_range
        self.speed = speed

    def get_command(self):

        # get lasers data
        laser_ranges = self._env.get_laserranges()
        laser_ranges = [self._laser_range - i for i in laser_ranges]

        if sum(laser_ranges[:5]) > self._threshold:
            if self._verbose:
                print("WALL L", sum(laser_ranges[:5]), sum(laser_ranges[5:]))
            return [-self.speed, self.speed]

        elif sum(laser_ranges[5:]) > self._threshold:
            if self._verbose:
                print("WALL R", sum(laser_ranges[:5]), sum(laser_ranges[5:]))
            return [self.speed, -self.speed]

        else:
            if self._verbose:
                print("NO WALL ", sum(laser_ranges[:5]), sum(laser_ranges[5:]))
            return [self.speed, self.speed]

    def reset(self):
        pass
