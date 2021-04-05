from controllers.follow_wall import Follow_wallController
from controllers.forward import ForwardController
import time

from time import sleep
import gym
from iRobot_gym.envs import SimpleNavEnv
import random
from numpy import array
import numpy as np
import matplotlib.pyplot as plt
import os
import csv

ListePosition = []


class SimEnv():

    def __init__(self, env, sleep_time, display):
        self.env = gym.make(env)
        self.env.reset()
        self.sleep_time = sleep_time
        self.display = display

        self.obs, self.rew, self.done, self.info = self.env.step(
            dict([('motor', array([0, 0]))]))

    def mouvement(self, c, n=1):
        for _ in range(n):
            obs, rew, done, info = self.env.step(c)
            #print("Valeur de obs :" + str(obs))
            #print("Valeur de rew :" + str(rew))
            #print("Valeur de done :" + str(done))
            #print("Valeur de info :" + str(info))
            if(self.display):
                time.sleep(self.sleep_time)
            if self.done:
                break
            self.env.render()
            self.i += 1

        return obs, rew, done, info

    def start(self):

        # initialize controllers
        forward = ForwardController(self.env, verbose=True)
        wall = Follow_wallController(self.env, verbose=True)
        self.controller = wall

        # start timers
        then = time.time()
        self.i = 0

        while not self.done and self.i < 1000:
            command = self.controller.get_command()
            # print(command)
            self.obs, self.rew, self.done, self.info = self.mouvement(command)
            x, y, z, roll, pitch, yaw = self.info['pose']
            print(x, y, z, roll, pitch, yaw)
            ListePosition.append([self.i, x, y, z, roll, pitch, yaw,
                                  self.info["progress"], self.obs['lidar'][::len(self.obs['lidar'])-1]])
            self.controller.reset()

        now = time.time()
        print("%d timesteps took %f seconds" % (self.i, now - then))
        input("Press Enter to continue...")
        self.env.close()


if __name__ == "__main__":
    env1 = 'Kitchen-v0'
    env2 = 'Maze_hard-v0'
    sleep_time = 0.01
    display = True
    simEnv = SimEnv(env2, sleep_time, display)
    simEnv.start()

    with open('results/result.csv', 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["steps", "x", "y", "z", "roll", "pitch", "yaw",
                        "distance_to_obj", "lidar"])
        writer.writerows(ListePosition)
