import random
from numpy import array

speed = 0.0081


class BraitenbergController:

    def __init__(self, env, verbose=False):

        self.env = env
        self.verbose = verbose

        # behavioral parameters
        self.reactivity = 0.8
        self.speed = speed

    def get_command(self):

        # get lasers data
        laserRanges = self.env.get_laserranges()
        laserRanges = [1 - i for i in laserRanges]
        # print(laserRanges)

        Sr = sum(laserRanges[:5])
        Sl = sum(laserRanges[5:])

        left = self.speed*(1+self.reactivity*(Sl-Sr))
        right = self.speed*(1+self.reactivity*(Sr-Sl))

        if self.verbose:
            print("Sr:", Sr, "Sl:", Sl, "left:",
                  left, 'right:', right)
        return [left, right]

    def reset(self):
        self.reactivity = 0.8
        self.speed = speed
