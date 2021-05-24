"""The Forward controller class."""
import random


class ForwardController:
    """
    With this controller, the agent will always go forward when possible,
    otherwise it will react accordingly to the obstacle or wall ahead
    """

    def __init__(self, env, verbose=False):

        self.env = env
        self.verbose = verbose

        # behavioral parameters
        self.dist_tooClose = 0.4
        self.wall_tooCloseF = False
        self.wall_tooCloseR = False
        self.wall_tooCloseL = False
        self.v_forward = 1
        self.v_turn = 0.4
        self.right = [self.v_turn+0.1, -self.v_turn]
        self.left = [-self.v_turn, self.v_turn+0.1]
        self.forward = [self.v_forward, self.v_forward]

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
                elif i in range(4, 6):
                    self.wall_tooCloseF = True
                elif i in range(6, 10):
                    self.wall_tooCloseR = True

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

        if self.verbose:
            print(f"Chosen action : {c}")

        return c

    def reset(self):
        self.dist_tooClose = 0.4
        self.wall_tooCloseF = False
        self.wall_tooCloseR = False
        self.wall_tooCloseL = False
        self.v_forward = 1
        self.v_turn = 0.4
        self.right = [self.v_turn+0.1, -self.v_turn]
        self.left = [-self.v_turn, self.v_turn+0.1]
        self.forward = [self.v_forward, self.v_forward]
