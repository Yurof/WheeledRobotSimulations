import os
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import argparse
import csv

from iRobot_gym.bullet.configs import ScenarioSpec


def plot_position(name, ListeResults, startx, starty, goalx, goaly, goalsize, ratio):
    img = plt.imread(
        f'{base_path}/../pybullet/models/scenes/{name}/{name}.pbm')
    nb_rapport = img.shape[0]/ratio

    # plt.annotate("Goal", xy=(goalx*nb_rapport, goaly*nb_rapport),
    #              xytext=(goalx*nb_rapport, goaly*nb_rapport-5))
    plt.plot(goalx*nb_rapport, goaly*nb_rapport,
             'go', markersize=goalsize*nb_rapport)

    plt.annotate("Start", xy=(startx*nb_rapport, starty*nb_rapport),
                 xytext=(startx*nb_rapport-5, starty*nb_rapport-15))
    plt.plot(startx*nb_rapport, starty*nb_rapport, 'rx')

    plt.imshow(np.flipud(img), origin='lower')
    for s in ListeResults:
        df = pd.read_csv(f'{base_path}/{name}/{s}.csv')
        x = df.x
        y = df.y
        plt.plot(x*nb_rapport, y*nb_rapport, label=s, alpha=0.5)
        plt.legend(loc='best')
        plt.axis('off')
    plt.savefig("{name}.png", bbox_inches='tight', dpi=300)
    plt.show()


def plot_dist_to_target(name, ListeResults):
    L_df = [pd.read_csv(
        f'{base_path}/{name}/{s}.csv') for s in ListeResults]
    L_x = [df.x for df in L_df]
    L_y = [df.y for df in L_df]
    L_s = [df.steps for df in L_df]
    L_d = [df.distance_to_obj for df in L_df]

    plt.subplot(2, 2, 1)
    for i in range(len(ListeResults)):
        plt.plot(L_s[i], L_x[i], label=ListeResults[i], alpha=0.5)
    plt.legend(loc='best')
    plt.xlabel("step")
    plt.ylabel("x")

    plt.subplot(2, 2, 2)
    for i in range(len(ListeResults)):
        plt.plot(L_s[i], L_y[i], label=ListeResults[i], alpha=0.5)
    plt.legend(loc='best')
    plt.xlabel("step")
    plt.ylabel("y")

    plt.subplot(2, 2, 3)
    for i in range(len(ListeResults)):
        plt.plot(L_s[i], L_d[i], label=ListeResults[i], alpha=0.5)
    plt.legend(loc='best')
    plt.xlabel("step")
    plt.ylabel("distance to objectif")
    plt.savefig("{name}-dist.png", bbox_inches='tight', dpi=300)
    plt.show()


if __name__ == "__main__":
    base_path = os.path.dirname(os.path.abspath(__file__))

    parser = argparse.ArgumentParser(
        description='Launch pybullet simulation run.')
    parser.add_argument('--env', type=str, default="maze_hard",
                        help='environnement: "kitchen", "maze_hard", "race_track"')
    parser.add_argument('--listPlot', type=str, default="braitenberg rule_based",
                        help='listPlot')
    args = parser.parse_args()
    env = args.env
    listPlot = args.listPlot.split()

    config = ScenarioSpec()
    config.load(
        f'{base_path}/../pybullet/configuration/scenarios/{env}.yml')

    sx, sy, _ = config.agents.starting_position
    gx, gy, _ = config.world.goal.goal_position
    gs = config.world.goal.goal_size
    scale = config.world.scale

    plot_position(env, listPlot, startx=sx,
                  starty=sy, goalx=gx, goaly=gy, goalsize=gs, ratio=scale)

    plot_dist_to_target(env, listPlot)
