#!/usr/bin/python
# -*- coding: utf-8 -*-
import gym
import gym_fastsim
import time
import os
import sys
import csv
import argparse
from controllers.follow_wall import FollowWallController
from controllers.forward import ForwardController
from controllers.rulebased import RuleBasedController
from controllers.braitenberg import BraitenbergController
from controllers.novelty_ctr import NoveltyController


class SimEnv():
    """This is the main class that runs the fastsim simulation daccording to the arguments.

    Attributes:
        _env: The actual environnment.
        _sleep_time: float, representing the sleep time between each step.
        _ctr: string, the name of the controller.
        _verbose: bool, activate verbose or not.
        _i: int, iterator for steps.
        _liste_position: list of tuples, values to save in the csv file.
    """

    def __init__(self):
        self._sleep_time = args.sleep_time
        self._verbose = args.verbose
        self._display = args.display
        self._ctr = args.ctr
        self._i = 0
        self._Liste_position = []

        if args.env == "kitchen":
            self._env = gym.make("kitchen-v1")
        elif args.env == "maze_hard":
            self._env = gym.make("maze-v0")
        elif args.env == "race_track":
            self._env = gym.make("race_track-v0")
        self._env.reset()
        self.map_size = self._env.get_map_size()
        self.obs, self.rew, self.done, self.info = self._env.step([0, 0])

        # initialize controllers
        if self._ctr == "forward":
            self._controller = ForwardController(
                self._env, verbose=self._verbose)
        elif self._ctr == "wall":
            self._controller = FollowWallController(
                self._env, verbose=self._verbose)
        elif self._ctr == "rule":
            self._controller = RuleBasedController(
                self._env, verbose=self._verbose)
        elif self._ctr == "braitenberg":
            self._controller = BraitenbergController(
                self._env, verbose=self._verbose)
        elif self._ctr == "novelty":
            self._controller = NoveltyController(
                self._env, args.file_name, verbose=self._verbose)
        else:
            print("\nNo controller named", self._ctr)
            sys.exit()

        if(self._display):
            self._env.enable_display()

    def _movement(self, action, nbr=1):
        for _ in range(nbr):
            obs, rew, done, info = self._env.step(action)
            #print("Valeur de obs :" + str(obs))
            #print("Valeur de rew :" + str(rew))
            #print("Valeur de done :" + str(done))
            #print("Valeur de info :" + str(info))

            print(self._i, end='\r')
            self._i += 1

            if(self._display):
                time.sleep(self._sleep_time)

            if args.save_res:
                x, y, theta = info['robot_pos']
                self._Liste_position.append(
                    [self._i, x, (self.map_size-y), theta, info["dist_obj"], obs])
            if done:
                break
            self._env.render()

        return obs, rew, done, info

    def start(self):
        """Forward the simulation until its complete."""
        while not self.done and self._i < 5000:
            try:
                command = self._controller.get_command()
                self.obs, self.rew, self.done, self.info = self._movement(
                    command)
                self._controller.reset()

            except KeyboardInterrupt:
                print(' The simulation was forcibly stopped.')
                break

        print("Number of steps:", self._i)
        self._env.close()

    def save_result(self):
        """Save the simulation data in a csv file in the folder
        corresponding to the controller and name it accordingly.
        """
        base_path = os.path.dirname(os.path.abspath(__file__))
        path = f'{base_path}/../results/{args.env}/fastsim_{args.ctr}_'
        i = 1
        if os.path.exists(path+str(i)+".csv"):
            while os.path.exists(path+str(i)+".csv"):
                i += 1
        with open(path+str(i)+".csv", 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["steps", "x", "y", "roll",
                             "distance_to_obj", "laser"])
            writer.writerows(self._Liste_position)
            print("\ndata saved in:", file.name)


def main():
    sim_env = SimEnv()
    sim_env.start()
    if args.save_res:
        sim_env.save_result()


if __name__ == "__main__":

    parser = argparse.ArgumentParser(
        description='Launch fastsim simulation run.')
    parser.add_argument('--env', type=str, default="maze_hard",
                        help='choose between kitchen, maze_hard and race_track')
    parser.add_argument('--ctr', type=str, default="novelty",
                        help='choose between wall, rule, braitenberg and novelty')
    parser.add_argument('--sleep_time', type=float, default=0.00,
                        help='sleeping time between each step')
    parser.add_argument('--display', type=bool, default=True,
                        help='True or False')
    parser.add_argument('--save_res', type=bool, default=True,
                        help='save the result in a csv file: True or False')
    parser.add_argument('--verbose', type=bool, default=False,
                        help='verbose for controller: True or False')
    parser.add_argument('--file_name', type=str,
                        default='NoveltyFitness/9/maze_nsfit9-gen38-p0', help='file name of the invidual to load if ctr=novelty')
    args = parser.parse_args()
    main()
