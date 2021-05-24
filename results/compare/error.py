import os
import argparse
import csv
import pandas as pd
from sklearn.metrics import mean_squared_error

base_path = os.path.dirname(os.path.abspath(__file__))


def compute_error(criteria, file_name):
    # assuming ListeResults is length 2 and bullet is the ground truth
    bullet_index = 0
    fastsim_index = (bullet_index + 1) % 2
    path = f'{base_path}/results/individuals/{criteria}'
    df1 = pd.read_csv(f'{path}/FastSim/{file_name}.csv')
    df2 = pd.read_csv(f'{path}/PyBullet/{file_name}.csv')
    dfs = [df1, df2]

    # if different number of steps
    min_count = min([len(df.index) for df in dfs])

    bullet = list(zip(dfs[bullet_index].x[:min_count],
                      dfs[bullet_index].y[:min_count]))
    fastsim = list(zip(dfs[fastsim_index].x[:min_count],
                       dfs[fastsim_index].y[:min_count]))

    with open(path+"/results.csv", 'a', encoding='UTF8') as file:
        writer = csv.writer(file)
        writer.writerow([mean_squared_error(bullet, fastsim)])
    return mean_squared_error(bullet, fastsim)

if __name__ == "__main__":

    parser = argparse.ArgumentParser(
        description='Launch pybullet simulation run.')
    # "forward", "wall", "rule", "brait"
    parser.add_argument('--file_name', type=str, default="fastsim_brait_10000_steps bullet_brait_10000_steps",
                        help='file_name')
    parser.add_argument('--criteria', type=str,
                        default='null', help='choose between "Fitness", "NoveltySearch", "NoveltyFitness"')


    args = parser.parse_args()
    file_name = args.file_name
    criteria = args.criteria
    print("Error between fastsim and bullet : ", compute_error(criteria, file_name))