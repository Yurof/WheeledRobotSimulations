import random
from numpy import array


class ForwardController:
    """
    With this controller, the agent will always go forward when possible,
    otherwise it will react accordingly to the obstacle or wall ahead
    """

    def __init__(self, env, verbose=False):

        self.env = env
        self.verbose = verbose

        # behavioral parameters
        self.dist_tooClose = 1
        self.wall_tooCloseF = False
        self.wall_tooCloseR = False
        self.wall_tooCloseL = False
        self.right = [-1, 1]
        self.left = [1, -1]
        self.forward = [0.5, 0.5]

        # there is this case where the agent might be stuck and alternate
        # endlessly between left and right, so we add an additional parameter
        # to try to prevent that
        self.old_c = self.forward

    def get_command(self):

        # command to return at the end of this function
        c = self.forward

        # get lasers data
        laserRanges = self.env.get_laserranges()

        for i in range(len(laserRanges)):
            if laserRanges[i] < self.dist_tooClose:
                # assuming there are 4 radars on the left, 2 on the front,
                # 4 on the right
                if i in range(4):
                    self.wall_tooCloseL = True
                if i in range(4, 6):
                    self.wall_tooCloseF = True
                if i in range(6, 10):
                    self.wall_tooCloseR = True

        # get bumpers data

        # we adapt our policy based on what we have detected
        if self.wall_tooCloseF:
            if self.verbose:
                print("WALL F")
            # randomly turn right or left
            if random.random() < 0.5:
                c = self.right
            else:
                c = self.left

        elif self.wall_tooCloseL:
            if self.verbose:
                print("WALL L")
            # turn right
            c = self.right

        elif self.wall_tooCloseR:
            if self.verbose:
                print("WALL R")
            # turn left
            c = self.left
        print(c, self.right, self.old_c, self.left, self.right)
        if (c == self.right and self.old_c == self.left) or (c == self.left and self.old_c == self.right):
            c = self.old_c
        self.old_c = c

        if self.verbose:
            print(f"Chosen action : {c}")

        return c

    def reset(self):
        self.dist_tooClose = 1
        self.wall_tooCloseF = False
        self.wall_tooCloseR = False
        self.wall_tooCloseL = False
        self.right = [-1, 1]
        self.left = [1, -1]
        self.forward = [0.5, 0.5]
