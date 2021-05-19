from fixed_structure_nn_numpy import SimpleNeuralControllerNumpy
import gym
from iRobot_gym.envs import SimpleNavEnv
import matplotlib.pyplot as plt
import os
import numpy as np
import pickle
import time
from deap import *

from deap import algorithms
from deap import base
from deap import benchmarks
from deap import creator
from deap import tools
import argparse
import array
import csv

base_path = os.path.dirname(os.path.abspath(__file__))

creator.create("MyFitness", base.Fitness, weights=(-1.0,))
creator.create("Individual", array.array, typecode="d",
               fitness=creator.MyFitness, strategy=None)
creator.create("Strategy", array.array, typecode="d")

env = gym.make('maze_hard-v0')

done = False

nn_size = [12, 2, 2, 10]
nbstep = 10000
render = True
f = open(f"{base_path}/../../results/pareto/paretoHOF4-1.pkl", "rb")
genotype = pickle.load(f)
# print(genotype)

nn = SimpleNeuralControllerNumpy(*nn_size)
nn.set_parameters(genotype)
observation = env.reset()
print("obsss", observation)
old_pos = None
total_dist = 0
start = time.time()

ListePosition = []


def save_result(name, controller):
    base_path = os.path.dirname(os.path.abspath(__file__))
    path = f'{base_path}/../../results/{name}/bullet_{controller}_'
    i = 1
    if os.path.exists(path+str(i)+".csv"):
        while os.path.exists(path+str(i)+".csv"):
            i += 1
    with open(path+str(i)+".csv", 'w', newline='') as file:
        writer = csv.writer(file)
        print("\ndata saved as ", file)
        writer.writerow(["steps", "x", "y", "z", "roll", "pitch", "yaw",
                         "distance_to_obj", "lidar"])
        writer.writerows(ListePosition)


print("\nobservation\n", observation)
if render:
    env.render()
for t in range(nbstep):
    time.sleep(0.0001)
    action = nn.predict(observation)
    # print("\n", t)
    # print("action ", action)
    observation, reward, done, info = env.step(action)
    x, y, z, roll, pitch, yaw = info['pose']
    ListePosition.append([t, x, y, z, roll, pitch, yaw,
                          info["progress"], observation])
    # print("observation", observation[-2:])
    # print("pose ", info['pose'][:2], info['pose'][5])

    if(done):
        break
        print("\n done time ", time.time()-start)

save_result("maze_hard", "genotype")
env.close()
