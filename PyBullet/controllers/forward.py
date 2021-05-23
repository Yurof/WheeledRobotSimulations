import random


class ForwardController:
    """
    With this controller, the agent will always go forward when possible,
    otherwise it will react accordingly to the obstacle or wall ahead
    """

    def __init__(self, env, dist_too_close=0.4, v_forward=1, v_turn=0.4, verbose=False):

        self._env = env
        self._verbose = verbose

        # behavioral parameters
        self._dist_too_close = dist_too_close
        self._v_forward = v_forward
        self._v_turn = v_turn

        self._wall_too_close_f = False
        self._wall_too_close_r = False
        self._wall_too_close_l = False

    def get_command(self):

        # get lasers data
        laser_ranges = self._env.get_laserranges()

        for i in range(len(laser_ranges)):
            if laser_ranges[i] < self._dist_too_close:
                # assuming there are 4 radars on the left, 2 on the front,
                # 4 on the right
                if i in range(4):
                    self._wall_too_close_l = True
                elif i in range(4, 6):
                    self._wall_too_close_f = True
                elif i in range(6, 10):
                    self._wall_too_close_r = True

        # we adapt our policy based on what we have detected
        if self._wall_too_close_f:
            if self._verbose:
                print("WALL F")
            if random.random() < 0.5:  # randomly turn right or left
                return [self._v_turn+0.1, -self._v_turn]
            return [-self._v_turn, self._v_turn+0.1]

        elif self._wall_too_close_l:
            if self._verbose:
                print("WALL L")
            return [self._v_turn+0.1, -self._v_turn]  # turn right

        elif self._wall_too_close_r:
            if self._verbose:
                print("WALL R")
            return [-self._v_turn, self._v_turn+0.1]  # turn left

        return [self._v_forward, self._v_forward]

    def reset(self):
        self._wall_too_close_f = False
        self._wall_too_close_r = False
        self._wall_too_close_l = False
