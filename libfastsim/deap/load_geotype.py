import cma
import gym
import gym_fastsim
from deap import *
import numpy as np
from fixed_structure_nn_numpy import SimpleNeuralControllerNumpy
from scipy.spatial import KDTree

from deap import algorithms
from deap import base
from deap import benchmarks
from deap import creator
from deap import tools
import argparse

import array
import random
import operator
import math

import os
from scoop import futures
import csv
import time
import pickle

ListePosition = []


def eval_nn(genotype, nbstep=10000, render=False, name="", nn_size=[12, 2, 2, 10]):
    nn = SimpleNeuralControllerNumpy(*nn_size)
    nn.set_parameters(genotype)
    observation = env.reset()
    old_pos = None
    total_dist = 0
    start = time.time()
    # print("géno ", genotype)
    # print("\nobservation 0\n", observation)
    if (render):
        f = open("traj"+name+".log", "w")
    for t in range(nbstep):
        # time.sleep(0.1)
        if render:
            env.render()
            env.enable_display()

        action = nn.predict(observation)/105
        # print("action ", action)
        observation, reward, done, info = env.step(action)
        x, y, theta = info['robot_pos']
        ListePosition.append(
            [t, x, (10-y), theta, info["dist_obj"], observation])
        pos = info["robot_pos"][:2]
        print("observation", observation)
        # print("pos", pos[0], 10-pos[1], info["robot_pos"][2])

        if(render):
            f.write(" ".join(map(str, pos))+"\n")
        if (old_pos is not None):
            d = math.sqrt((pos[0]-old_pos[0])**2+(pos[1]-old_pos[1])**2)
            total_dist += d
        old_pos = list(pos)
        if(done):
            break
            print("\n done time ", time.time()-start)
    if (render):
        f.close()
    dist_obj = info["dist_obj"]
    # print("End of eval, total_dist=%f"%(total_dist))
    # Remarque: les positions et distances sont arrondis à 2 décimales pour éviter le surapprentissage et le maintien dans le front de pareto de solutions ne différant que
    # de décimales plus éloignées (variante FIT+NS)
    rpos = [round(x, 2) for x in pos]
    # if round(dist_obj, 2) == 0:
    #     return 0, rpos
    # else:
    #     return 1, rpos
    return round(dist_obj, 2), rpos


def save_result(name, controller):
    base_path = os.path.dirname(os.path.abspath(__file__))
    path = f'{base_path}/../../results/{name}/fastsim_{controller}_'
    i = 1
    if os.path.exists(path+str(i)+".csv"):
        while os.path.exists(path+str(i)+".csv"):
            i += 1
    with open(path+str(i)+".csv", 'w', newline='') as file:
        writer = csv.writer(file)
        print("\ndata saved as ", file)
        writer.writerow(["steps", "x", "y", "theta",
                         "distance_to_obj", "laser"])
        writer.writerows(ListePosition)


if (__name__ == "__main__"):
    env = gym.make("maze_hard-v0")
    base_path = os.path.dirname(os.path.abspath(__file__))
    creator.create("MyFitness", base.Fitness, weights=(-1.0,))
    creator.create("Individual", array.array, typecode="d",
                   fitness=creator.MyFitness, strategy=None)
    creator.create("Strategy", array.array, typecode="d")
    f = open(f"{base_path}/../../results/pareto/paretoHOF4-1.pkl", "rb")
    new_dict = pickle.load(f)
    # print(new_dict)
    f.close()
    # p = f.read()
    # print(p)
    eval_nn(new_dict, render=True)
    save_result("maze_hard", "genotype")
