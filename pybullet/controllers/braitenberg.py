
class BraitenbergController:

    def __init__(self, env, laser_range=1, speed=1, reactivity=0.8, verbose=False):
        self._env = env
        self._verbose = verbose
        self._reactivity = reactivity
        self._speed = speed
        self._laser_range = laser_range

    def get_command(self):

        # get lasers data
        laser_ranges = self._env.get_laserranges()
        laser_ranges = [self._laser_range - i for i in laser_ranges]

        sr = sum(laser_ranges[:5])
        sl = sum(laser_ranges[5:])

        left = self._speed*(1+self._reactivity*(sl-sr))
        right = self._speed*(1+self._reactivity*(sr-sl))

        if self._verbose:
            print("Sr:", sr, "Sl:", sl, "left:", left, 'right:', right)

        return [left, right]

    def reset(self):
        pass
