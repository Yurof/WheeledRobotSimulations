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

        # initialize controllers
        forward = ForwardController(self.env, verbose=False)
        wall = Follow_wallController(self.env, verbose=False)
        rule = RuleBasedController(self.env, verbose=False)
        brait = BraitenbergController(self.env, verbose=False)
        self.controller = brait

        # start timers
        then = time.time()
        self.i = 0

        while not self.done:
            try:
                command = self.controller.get_command()
                self.obs, self.rew, self.done, self.info = self.mouvement(command)
                x, y, z= self.info['robot_pos']
                ListePosition.append([self.i, x/66, (1000-y)/66,self.info["dist_obj"]])
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
    while os.path.exists(path+str(i)+".csv"):
        i += 1
    with open(path+str(i)+".csv",'w', newline='') as file:
        writer = csv.writer(file)
        print("\ndata saved as ", file)
        writer.writerow(["steps", "x", "y", "z", "roll", "pitch", "yaw",
                        "distance_to_obj", "lidar"])
        writer.writerows(ListePosition)


if __name__ == "__main__":
    env1 = 'kitchen-v1'
    env2 = 'maze-v0'
    env3 = 'race_track-v0'
    sleep_time = 0.001
    display = True
    simEnv = SimEnv(env3, sleep_time, display)
    simEnv.start()
    print(os.path.dirname(os.path.abspath(__file__)))
    save_result("race_track", 'brait')
