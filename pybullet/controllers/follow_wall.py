"""The Follow wall controller class."""

from .forward import ForwardController


class FollowWallController:
    """The agent follows a wall while maintaining a distance specified by the user
    if no wall is seen, it will search for one using the forward controller.
    """

    def __init__(self, env, dist_too_close=0.4, dist_too_far=0.7, dist_obstacle=0.7, v_forward=1, v_turn=0.4, verbose=False):
        """Inits FollowWallController with the attributes values"""
        self.env = env
        self._verbose = verbose
        self.forwardcontroller = ForwardController(env)

        self._dist_too_close = dist_too_close
        self._dist_too_far = dist_too_far
        self._dist_obstacle = dist_obstacle
        self._v_forward = v_forward
        self._v_turn = v_turn

        self.obstacle_front = False
        self.wall_too_close_r = False
        self.wall_too_close_l = False
        self.wall_r_ok = False
        self.wall_l_ok = False
        self.wall_too_far_r = False
        self.wall_too_far_l = False

        self.left = [self._v_turn+0.1, -self._v_turn]
        self.right = [-self._v_turn, self._v_turn+0.1]
        self.forward = [self._v_forward, self._v_forward]

    def get_command(self):

        # command to return at the end of this function

        min_dist_l = 1000
        min_dist_r = 1000

        # get lasers data
        laserRanges = self.env.get_laserranges()
        if self._verbose:
            print("laserrange", laserRanges)
        # assuming there are 10 lasers and
        # 5th and 6th laser are on the front
        for dist in laserRanges[4:6]:
            if dist < self._dist_obstacle:
                self.obstacle_front = True

        # 1st to 4th laser are on the left
        for dist in laserRanges[:4]:
            if min_dist_l > dist:
                min_dist_l = dist
            if dist < self._dist_too_close:
                self.wall_too_close_l = True
            elif dist < self._dist_too_far:
                self.wall_l_ok = True
            elif dist < 1:
                self.wall_too_far_l = True

        # 7th to 10th laser are on the right
        for dist in laserRanges[6:]:
            if min_dist_r > dist:
                min_dist_r = dist
            if dist < self._dist_too_close:
                self.wall_too_close_r = True
            elif dist < self._dist_too_far:
                self.wall_r_ok = True
            elif dist < 1:
                self.wall_too_far_r = True

        # we adapt our policy based on what we have detected
        if self.obstacle_front:
            if self._verbose:
                print("OBSTACLE FRONT")
            if min_dist_l < min_dist_r:
                return [-self._v_turn, self._v_turn+0.1]
            else:
                return [self._v_turn+0.1, -self._v_turn]

        elif self.wall_too_close_l and (not self.wall_too_close_r or min_dist_l < min_dist_r):
            if self._verbose:
                print("TOO CLOSE LEFT")
            return [-self._v_turn, self._v_turn+0.1]

        elif self.wall_too_close_r and (not self.wall_too_close_l or min_dist_r < min_dist_l):
            if self._verbose:
                print("TOO CLOSE RIGHT")
            return [self._v_turn+0.1, -self._v_turn]

        elif self.wall_l_ok or self.wall_r_ok:
            if self._verbose:
                print("NP")
            return [self._v_forward, self._v_forward]

        elif self.wall_too_far_l and (not self.wall_too_far_r or min_dist_l < min_dist_r):
            if self._verbose:
                print("TOO FAR LEFT")
            return [self._v_turn+0.1, -self._v_turn]

        elif self.wall_too_far_r and (not self.wall_too_far_l or min_dist_r < min_dist_l):
            if self._verbose:
                print("TOO FAR RIGHT")
            return [-self._v_turn, self._v_turn+0.1]

        else:
            if self._verbose:
                print("NO WALL")
            return self.forwardcontroller.get_command()

        return [self._v_forward, self._v_forward]

    def reset(self):
        self.forwardcontroller.reset()
        self.obstacle_front = False
        self.wall_too_close_r = False
        self.wall_too_close_l = False
        self.wall_r_ok = False
        self.wall_l_ok = False
        self.wall_too_far_r = False
        self.wall_too_far_l = False
