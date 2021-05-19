#!/usr/bin/python
# -*- coding: utf-8 -*-
import gym
import gym_fastsim
import time
import os
import csv
import argparse
from controllers.follow_wall import Follow_wallController
from controllers.forward import ForwardController
from controllers.rulebased import RuleBasedController
from controllers.braitenberg import BraitenbergController
from controllers.novelty_ctr import NoveltyController

ListePosition = []


class SimEnv():

    def __init__(self, env, ctr, sleep_time, display):
        if env == "kitchen":
            self.env = gym.make("kitchen-v1")
        elif env == "maze_hard":
            self.env = gym.make("maze_hard-v0")
        elif env == "race_track":
            self.env = gym.make("race_track-v0")
        self.env.reset()
        self.sleep_time = sleep_time
        self.display = display
        self.map_size = self.env.get_map_size()
        print(self.map_size)

        self.obs, self.rew, self.done, self.info = self.env.step([0, 0])

        # initialize controllers
        if ctr == "forward":
            self.controller = ForwardController(self.env, verbose=False)
        elif ctr == "wall":
            self.controller = Follow_wallController(self.env, verbose=False)
        elif ctr == "rule":
            self.controller = RuleBasedController(self.env, verbose=False)
        elif ctr == "brait":
            self.controller = BraitenbergController(self.env, verbose=False)
        elif ctr == "novelty":
            self.controller = NoveltyController(self.env)

        if(self.display):
            self.env.enable_display()

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

        return obs, rew, done, info

    def start(self):
        # start timers
        then = time.time()
        self.i = 0

        while not self.done:
            try:
                command = self.controller.get_command()
                self.obs, self.rew, self.done, self.info = self.mouvement(
                    command)
                x, y, theta = self.info['robot_pos']
                ListePosition.append(
                    [self.i, x, (self.map_size-y), theta, self.info["dist_obj"], self.obs])
                print(self.i)
                self.controller.reset()
                self.i += 1
            except KeyboardInterrupt:
                print('All done')
                break
        now = time.time()
        print("%d timesteps took %f seconds" % (self.i, now - then))
        self.env.close()


def save_result(name, controller):
    base_path = os.path.dirname(os.path.abspath(__file__))
    path = f'{base_path}/../results/{name}/fastsim_{controller}_'
    i = 1
    if os.path.exists(path+str(i)+".csv"):
        while os.path.exists(path+str(i)+".csv"):
            i += 1
    with open(path+str(i)+".csv", 'w', newline='') as file:
        writer = csv.writer(file)
        print("\ndata saved as ", file)
        writer.writerow(["steps", "x", "y", "roll",
                         "distance_to_obj", "lidar"])
        writer.writerows(ListePosition)


if __name__ == "__main__":

    parser = argparse.ArgumentParser(
        description='Launch fastsim simulation run.')
    # "kitchen", "maze", "race_track"
    parser.add_argument('--env', type=str, default="race_track",
                        help='choose between kitchen, maze and race_track')
    # "forward", "wall", "rule", "brait", "novelty
    parser.add_argument('--ctr', type=str, default="brait",
                        help='choose between forward, wall, rule, brait and novelty')
    parser.add_argument('--sleep_time', type=int, default=0.0001,
                        help='sleeping time between each step')
    parser.add_argument('--display', type=bool, default=False,
                        help='True or False')

    args = parser.parse_args()
    env = args.env
    ctr = args.ctr
    sleep_time = args.sleep_time
    display = args.display

    simEnv = SimEnv(env, ctr, sleep_time, display)
    simEnv.start()
    save_result(env, ctr)
