"""The Rule Based controller class."""


class RuleBasedController:
    """It's a Rule Based controller,
    it has a constant speed and turn right or left depanding of the lasers data.

    Attributes:
        _env: The actual environnment.
        _verbose: A boolean indicating if we want debug informations.
        _threshold: A float representing the threshold for a turn.
        _speed: A float between 0 and 1 representing the speed of the robot.
        _laser_range: A float indicating the range of the lasers.
    """

    def __init__(self, env, laser_range=1, speed=1/(117*2), threshold=1.2, verbose=False):
        """Inits RuleBasedController with the attributes values"""
        self._env = env
        self._verbose = verbose
        self._threshold = threshold
        self._laser_range = laser_range
        self.speed = speed

    def get_command(self):
        """Calculates the futur action of the robot.

        Returns:
            [(-1)^a*speed, (-1)^b*speed]: the action for the left and right wheel.
        """

        laser_ranges = self._env.get_laserranges()  # get lasers data
        n_rays = len(laser_ranges)//2  # number of lasers on the left and right
        # no walls detected => 0
        laser_ranges = [self._laser_range - i for i in laser_ranges]

        if self._verbose:
            print(sum(laser_ranges[:n_rays]), sum(laser_ranges[-n_rays:]))

        if sum(laser_ranges[:n_rays]) > self._threshold:
            if self._verbose:
                print("WALL L ")
            return [-self.speed, self.speed]

        if sum(laser_ranges[-n_rays:]) > self._threshold:
            if self._verbose:
                print("WALL R ")
            return [self.speed, -self.speed]

        if self._verbose:
            print("NO WALL ")
        return [self.speed, self.speed]

    def reset(self):
        pass
