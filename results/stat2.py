import os
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import csv

from iRobot_gym.bullet.configs import ScenarioSpec


def plot_position(name, ListeResults, startx, starty, goalx, goaly, goalsize, ratio):
    img = plt.imread(
        f'{base_path}/../pybullet/models/scenes/{name}/{name}.pbm')
    nb_rapport = img.shape[0]/ratio

    # plt.annotate("Goal", xy=(goalx*nb_rapport, goaly*nb_rapport),
    #              xytext=(goalx*nb_rapport, goaly*nb_rapport-5))
    # plt.plot(goalx*nb_rapport, goaly*nb_rapport,
    #          'go', markersize=goalsize*nb_rapport)

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
    plt.savefig("a.png", bbox_inches='tight', dpi=300)
    plt.show()


def plot_dist_to_target(name, ListeResults):
    L_df = [pd.read_csv(
        f'{base_path}/{name}/{s}.csv') for s in ListeResults]
    L_x = [df.x for df in L_df]
    L_y = [df.y for df in L_df]
    L_s = [df.steps for df in L_df]
    L_d = [df.distance_to_obj for df in L_df]

    # plt.subplot(2, 2, 1)
    # for i in range(len(ListeResults)):
    #     plt.plot(L_s[i], L_x[i], label=ListeResults[i], alpha=0.5)
    # plt.legend(loc='best')
    # plt.xlabel("step")
    # plt.ylabel("x")

    # plt.subplot(2, 2, 2)
    # for i in range(len(ListeResults)):
    #     plt.plot(L_s[i], L_y[i], label=ListeResults[i], alpha=0.5)
    # plt.legend(loc='best')
    # plt.xlabel("step")
    # plt.ylabel("y")

    # plt.subplot(2, 2, 3)
    for i in range(len(ListeResults)):
        plt.plot(L_s[i], L_d[i], label=ListeResults[i], alpha=0.5)
    plt.legend(loc='best')
    plt.xlabel("step")
    plt.ylabel("distance to objectif")
    plt.savefig("b.png", bbox_inches='tight', dpi=300)
    plt.show()


if __name__ == "__main__":
    base_path = os.path.dirname(os.path.abspath(__file__))

    scenario = 'race_track'
    #ListPlot = ['braitenberg', 'rule_based']
    # ListPlot = ['fastsim_forward_1']
    ListPlot = ['fastsim', 'bullet 100%', 'bullet 99%']
    ListPlot = ['fastsim_gen-38',  # 'fastsim_gen-150_', 'fastsim_gen-140_',
                # 'fastsim_gen-130_', 'fastsim_gen-120_', 'fastsim_gen-110_',
                # 'fastsim_gen-100_',  'fastsim_gen-90_', 'fastsim_gen-80_',
                # 'fastsim_gen-70_', 'fastsim_gen-60_', 'fastsim_gen-50_','fastsim_gen-40_',
                'fastsim_gen-30', 'fastsim_gen-20', 'fastsim_gen-10']

    scenario = 'maze_hard'
    ListPlot = ['braitenberg', 'rule_based']

    # scenario = 'kitchen'
    #ListPlot = ['fastsim_braitenberg_1', 'fastsim_rule_1']

    config = ScenarioSpec()
    config.load(
        f'{base_path}/../pybullet/configuration/scenarios/{scenario}.yml')

    sx, sy, _ = config.agents.starting_position
    gx, gy, _ = config.world.goal.goal_position
    gs = config.world.goal.goal_size
    scale = config.world.scale

    # ListPlot = ['fastsim_rule_1', 'fastsim_rule_3']
    # ListPlot = ['fastsim_brait_1', 'bullet_brait_1']
    # ListPlot = ['fastsim_genotype_1', 'bullet_genotype_1']
    # ListPlot = ['fastsim_ns-gen10_1', 'fastsim_ns-gen20_1', 'fastsim_ns-gen30_1',
    #             'fastsim_ns-gen40_1', 'fastsim_ns-gen50_1', 'fastsim_ns-gen60_1']

    plot_position(scenario, ListPlot, startx=sx,
                  starty=sy, goalx=gx, goaly=gy, goalsize=gs, ratio=scale)

    plot_dist_to_target(scenario, ListPlot)
