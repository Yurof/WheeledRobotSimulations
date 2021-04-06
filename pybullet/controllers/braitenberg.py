import random
from numpy import array


class BraitenbergController:

    def __init__(self, env, verbose=False):

        self.env = env
        self.verbose = verbose

        # behavioral parameters
        self.reactivity = 0.5
        self.speed = 0.7

    def get_command(self):

        # get lasers data
        laserRanges = self.env.get_laserranges()
        laserRanges = [1 - i for i in laserRanges]

        Sr = sum(laserRanges[:5])
        Sl = sum(laserRanges[5:])

        right = self.speed*(1+self.reactivity*(Sl-Sr))
        left = self.speed*(1+self.reactivity*(Sr-Sl))

        if self.verbose:
            print("Sr:", Sr, "Sl:", Sl, "left:",
                  left, 'right:', right, end='\r')
        return [left, right]

    def reset(self):
        self.reactivity = 1.2
        self.speed = 0.7
