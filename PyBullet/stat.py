import os
import matplotlib.pyplot as plt
import numpy as np
import argparse
# import RandomMouv_bullet

import csv

import pandas as pd


base_path = os.path.dirname(os.path.abspath(__file__))


def plot_position(name, ListeResults, startx, starty, goalx, goaly, ratio):
    img = plt.imread(f'{base_path}/models/scenes/{name}/{name}.pbm')
    nb_rapport = img.shape[0]/ratio

    plt.annotate("Goal", xy=(goalx*nb_rapport, goaly*nb_rapport),
                 xytext=(goalx*nb_rapport, goaly*nb_rapport))
    plt.plot(goalx*nb_rapport, goaly*nb_rapport, 'go', markersize=15)

    plt.annotate("Start", xy=(startx*nb_rapport, starty*nb_rapport),
                 xytext=(startx*nb_rapport-5, starty*nb_rapport-5))
    plt.plot(startx*nb_rapport, starty*nb_rapport, 'rx')

    plt.imshow(np.flipud(img), origin='lower')
    for s in ListeResults:
        df = pd.read_csv(f'{base_path}/../results/{name}/{s}.csv')
        x = df.x
        y = df.y
        plt.plot(x*nb_rapport, y*nb_rapport, label=s, alpha=0.5)
        plt.legend(loc='best')
    plt.show()


def plot_dist_to_target(name, ListeResults):
    L_df = [pd.read_csv(
        f'{base_path}/../results/{name}/{s}.csv') for s in ListeResults]
    L_x = [df.x for df in L_df]
    L_y = [df.y for df in L_df]
    L_s = [df.steps for df in L_df]
    L_d = [df.distance_to_obj for df in L_df]

    plt.subplot(2, 2, 1)
    for i in range(len(ListeResults)):
        plt.plot(L_s[i], L_x[i], label=ListeResults[i])
    plt.legend(loc='best')
    plt.xlabel("step")
    plt.ylabel("x")

    plt.subplot(2, 2, 2)
    for i in range(len(ListeResults)):
        plt.plot(L_s[i], L_y[i], label=ListeResults[i])
    plt.legend(loc='best')
    plt.xlabel("step")
    plt.ylabel("y")

    plt.subplot(2, 2, 3)
    for i in range(len(ListeResults)):
        plt.plot(L_s[i], L_d[i], label=ListeResults[i])
    plt.legend(loc='best')
    plt.xlabel("step")
    plt.ylabel("distance to objectif")
    print(max(L_df[0].lidar))
    plt.show()


"""
Ratio:
race_track =15
maze_hard =6.6
kitchen =5
"""
if __name__ == "__main__":

    parser = argparse.ArgumentParser(
        description='Launch pybullet simulation run.')
    # "kitchen", "maze_hard", "race_track"
    parser.add_argument('--env', type=str, default="race_track",
                        help='environnement')
    # "forward", "wall", "rule", "brait"
    parser.add_argument('--listPlot', type=str, default="fastsim_brait_1 bullet_brait_1 ",
                        help='listPlot')

    args = parser.parse_args()
    env = args.env
    listPlot = args.listPlot.split()
    plot_position(env, listPlot, startx=3,
                  starty=4, goalx=2, goaly=5, ratio=20)

    plot_dist_to_target(env, listPlot)
