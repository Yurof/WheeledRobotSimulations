"""The Braitenberg controller class."""


class BraitenbergController:
    """It's a braitenberg controller, it adapts its speed according
    to what the left and right sensors send back.

    Attributes:
        _env: The actual environnment.
        _verbose: A boolean indicating if we want debug informations.
        _reactivity: A float representing the reactivity of the robot.
        _speed: A float between 0 and 1 representing the speed of the robot.
        _laser_range: A float indicating the range of the lasers.
    """

    def __init__(self, env, laser_range=1, speed=1, reactivity=0.8, verbose=False):
        """Inits BraitenbergController with the attributes values"""
        self._env = env
        self._verbose = verbose
        self._reactivity = reactivity
        self._speed = speed
        self._laser_range = laser_range

    def get_command(self):
        """Calculates the futur action of the robot.

        Returns:
            [left, right]: the action for the left and right wheel.
        """

        laser_ranges = self._env.observation  # get lasers data
        n_rays = len(laser_ranges)//2  # number of lasers on the left and right
        # no walls detected => 0
        laser_ranges = [self._laser_range - i for i in laser_ranges]

        sr = sum(laser_ranges[:n_rays])
        sl = sum(laser_ranges[n_rays:])

        left = self._speed*(1+self._reactivity*(sl-sr))
        right = self._speed*(1+self._reactivity*(sr-sl))

        if self._verbose:
            print("Sr:", sr, "Sl:", sl, "left:", left, 'right:', right)

        return [left, right]

    def reset(self):
        pass
