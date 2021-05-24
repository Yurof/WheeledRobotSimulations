#!/usr/bin/python
# -*- coding: utf-8 -*-
import gym
import gym_fastsim
import time
import os
import csv
from controllers.follow_wall import FollowWallController
from controllers.forward import ForwardController
from controllers.rulebased import RuleBasedController
from controllers.braitenberg import BraitenbergController

ListePosition = []


class SimEnv():

    def __init__(self, env, sleep_time, display):
        self.env = gym.make(env)
        self.env.reset()
        self.sleep_time = sleep_time
        self.display = display
        self.map_size = self.env.get_map_size()
        if(self.display):
            self.env.enable_display()

        self.obs, self.rew, self.done, self.info = self.env.step([0, 0])

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

        while self.info['robot_pos'][0] < 100:
            try:
                self.obs, self.rew, self.done, self.info = self.mouvement([
                                                                          0.0001, 0.0001])
                x, y, theta = self.info['robot_pos']
                ListePosition.append(
                    [self.i, x, (self.map_size-y), theta, self.info["dist_obj"], self.obs])
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
                         "distance_to_obj", "laser"])
        writer.writerows(ListePosition)


if __name__ == "__main__":
    env1 = 'kitchen-v1'
    env2 = 'maze-v0'
    env3 = 'race_track-v0'
    sleep_time = 0.000001

    display = True
    simEnv = SimEnv(env3, sleep_time, display)
    simEnv.start()
    save_result("race_track", 'brait')
