from .forward import ForwardController


class FollowWallController:

    """

    The agent follows a wall while maintaining a distance specified by the user.

    If no wall is seen, it will search for one using the forward controller

    """

    def __init__(self, env, verbose=False):

        self.env = env

        self.verbose = verbose

        self.forwardcontroller = ForwardController(env)

        # behavioral parameters

        self.dist_tooClose = 0.4

        self.dist_tooFar = 0.9

        self.dist_obstacle = 0.5

        self.obstacleFront = False

        self.wall_tooCloseR = False

        self.wall_R_OK = False

        self.wall_tooFarR = False

        self.wall_tooCloseL = False

        self.wall_L_OK = False

        self.wall_tooFarL = False

        self.right = [1/117, -1/117]

        self.left = [-1/117, 1/117]

        self.forward = [1/117, 1/117]

        # there is this case where the agent might be stuck and alternate

        # endlessly between left and right, so we add an additional parameter

        # to try to prevent that

        self.old_c = self.forward

    def get_command(self):

        # command to return at the end of this function

        c = self.forward

        min_dist_L = 1000

        min_dist_R = 1000

        # get lasers data

        laserRanges = self.env.get_laserranges()

        # assuming there are 10 lasers and

        # 5th and 6th laser are on the front

        for dist in laserRanges[4:6]:

            if dist < self.dist_obstacle:

                self.obstacleFront = True

        # 1st to 4th laser are on the left

        for dist in laserRanges[:4]:

            if min_dist_L > dist:

                min_dist_L = dist

            if dist < self.dist_tooClose:

                self.wall_tooCloseL = True

            elif dist < self.dist_tooFar:

                self.wall_L_OK = True

            elif dist < 1:

                self.wall_tooFarL = True

        # 7th to 10th laser are on the right

        for dist in laserRanges[6:]:

            if min_dist_R > dist:

                min_dist_R = dist

            if dist < self.dist_tooClose:

                self.wall_tooCloseR = True

            elif dist < self.dist_tooFar:

                self.wall_R_OK = True

            elif dist < 1:

                self.wall_tooFarR = True

        # we adapt our policy based on what we have detected

        if self.obstacleFront:

            if self.verbose:

                print("OBSTACLE FRONT")

            c = self.forwardcontroller.get_command()

        elif self.wall_tooCloseL and (not self.wall_tooCloseR or min_dist_L < min_dist_R):

            if self.verbose:

                print("TOO CLOSE LEFT")

            c = self.right

        elif self.wall_tooCloseR and (not self.wall_tooCloseL or min_dist_R < min_dist_L):

            if self.verbose:

                print("TOO CLOSE RIGHT")

            c = self.left

        elif self.wall_L_OK or self.wall_R_OK:

            if self.verbose:

                print("NP")

            c = self.forward

        elif self.wall_tooFarL and (not self.wall_tooFarR or min_dist_L < min_dist_R):

            if self.verbose:

                print("TOO FAR LEFT")

            c = self.left

        elif self.wall_tooFarR and (not self.wall_tooFarL or min_dist_R < min_dist_L):

            if self.verbose:

                print("TOO FAR RIGHT")

            c = self.right

        else:

            if self.verbose:

                print("NO WALL")

            c = self.forwardcontroller.get_command()

        if (c == self.right and self.old_c == self.left) or \
                (c == self.left and self.old_c == self.right):
            c = self.old_c

        self.old_c = c

        if self.verbose:

            print(f"Chosen action : {c}")

        return c

    def reset(self):

        self.forwardcontroller.reset()

        self.dist_tooClose = 0.4

        self.dist_tooFar = 0.9

        self.dist_obstacle = 0.5

        self.obstacleFront = False

        self.wall_tooCloseR = False

        self.wall_R_OK = False

        self.wall_tooFarR = False

        self.wall_tooCloseL = False

        self.wall_L_OK = False

        self.wall_tooFarL = False

        self.right = [1/117, -1/117]

        self.left = [-1/117, 1/117]

        self.forward = [1/117, 1/117]
