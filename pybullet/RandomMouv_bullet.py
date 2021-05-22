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

env = gym.make('kitchen-v0')
##env = gym.make('Maze_hard-v0')

done = False
time_sleep = 0.01

obs = env.reset()

i = 0
action = [0, 0]
lidar_collision = 0.35
obs, rewards, done, states = env.step(action)
ListePosition = []


def mouvement(l, r, n):
    for k in range(n):
        obs, rewards, done, states = env.step([l, r])
        #print(obs['lidar'][len(obs['lidar'])//2], end="\r")
        sleep(time_sleep)
        image = env.render()
        # if states['wall_collision']==True:
        #     print("touche le mur")

        global i
        i += 1
        x, y, z, roll, pitch, yaw = states['pose']
        print(x, y, z, roll, pitch, yaw)
        ListePosition.append([i, x, y, z, roll, pitch, yaw,
                              states["dist_obj"], obs['lidar'][::len(obs['lidar'])-1]])

        print("Step  %d reward=%f robot position=%s progress=%f" % (
            i, rewards,  str(states["pose"][0:2]), states["dist_obj"]), end="\r")

        return obs, rewards, done, states


while not done and i < 1000:
    if obs['lidar'][len(obs['lidar'])//2] < lidar_collision-0.1:
        #print("avant ")
        obs, rewards, done, states = mouvement(-2, 2, 2)

    elif obs['lidar'][0] < lidar_collision:
        #print("gauche? ")
        obs, rewards, done, states = mouvement(2, -2, 1)

    elif obs['lidar'][-1] < lidar_collision:
        #print("droite? ")
        obs, rewards, done, states = mouvement(-2, 2, 1)
    else:
        obs, rewards, done, states = mouvement(
            random.uniform(-2, 5), random.uniform(-2, 5), 1)

print("Step  %d reward=%f robot position=%s progress=%f" %
      (i, rewards,  str(states["pose"][0:3]), states["dist_obj"]), end="\r")

env.close()

with open('results/result.csv', 'w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["steps", "x", "y", "z", "roll", "pitch", "yaw",
                     "distance_to_obj", "lidar"])
    writer.writerows(ListePosition)
