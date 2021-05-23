#!/usr/bin/python
# -*- coding: utf-8 -*-
import gym
import gym_fastsim
import time
import os
import csv
from controllers.follow_wall import Follow_wallController
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
                                                                          0.1, 0.1])
                x, y, theta = self.info['robot_pos']
                # print(self.obs)
                print(self.env.get_bumpers())
                ListePosition.append(
                    [self.i, x, (self.map_size-y), theta, self.info["dist_obj"], self.obs])
                self.i += 1
            except KeyboardInterrupt:
                print('All done')
                break
        now = time.time()
        print("%d timesteps took %f seconds" % (self.i, now - then))
        self.env.close()

    def rotation(self):

        self.i = 0
        # start timers
        then = time.time()
        moyenn = []
        while not self.done and (self.info['robot_pos'][2] > 0 or self.i < 2):
            try:
                self.obs, self.rew, self.done, self.info = self.mouvement([
                                                                          -0.0095, 0.0095])
                x, y, theta = self.info['robot_pos']
                print(self.i, theta)
                print(self.obs)
                print(self.env.get_bumpers())
                self.i += 1
                time.sleep(self.sleep_time)
                # print(self.info['velocity'][0])
            except KeyboardInterrupt:
                print('All done')
                break
        now = time.time()
        print(self.i)
        print("\ntook %f seconds of execution\n" % (now - then))
        print("\n ")
        self.env.close()


if __name__ == "__main__":
    env1 = 'kitchen-v1'
    env2 = 'maze-v0'
    env3 = 'race_track-v0'
    sleep_time = 0.01

    display = True
    simEnv = SimEnv(env3, sleep_time, display)
    simEnv.start()
<<<<<<< Updated upstream:libfastsim/speed_test.py
=======
    #save_result("race_track", 'brait')
>>>>>>> Stashed changes:libfastsim/speedtest.py
