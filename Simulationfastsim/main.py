#!/usr/bin/python
# -*- coding: utf-8 -*-
import gym, gym_fastsim
import time
import random
import sys
import numpy as np

class SimEnv():
    
    def __init__(self, env, sleep_time, display):
        self.env = gym.make(env)
        self.env.reset()
        self.sleep_time = sleep_time
        self.display = display
        if(self.display):
            self.env.enable_display()
        self.then = time.time()
        self.i = 0 
        self.obs, self.rew, self.done, self.info = self.env.step([0,0])
        self.diametre_goal= self.env.goal.get_diam()

    def mouvement(self, l, r, n):
        for k in range(n):
            self.obs, self.rew, self.done, self.info = self.env.step([l,r])
            print("Valeur de obs :" + str(self.obs))
            print("Valeur de rew :" + str(self.rew))
            print("Valeur de done :" + str(self.done))
            print("Valeur de info :" + str(self.info))
            oldDist = self.info["dist_obj"]
            if(self.display):
                time.sleep(self.sleep_time)
            if self.done:
                break
            self.env.render()
            global i
            self.i+= 1
            # print("Step %d Obs=%s reward=%f  dist. to objective=%f  robot position=%s" % (i, str([round(num,1) for num in o]),r, info["dist_obj"], str(info["robot_pos"]) ) , end="\r" )
            return self.obs, self.rew, self.done, self.info
    
    def naifBumper(self):
        dist = np.inf
        
        while dist > self.diametre_goal :
            # si aucun element devant, avancer
            if not (self.obs[-3] or self.obs[-2]):
                self.obs, self.rew, self.done, self.info = self.mouvement(1, 1, 1)
                newDist = self.info["dist_obj"]
                # si on s'eloigne, tourner a droite
                if newDist > dist:
                    for i in range(24):
                        self.mouvement(-2, 2, 1)
                dist = newDist
            
            # si element devant, tourner a droite
            elif self.obs[-3] and self.obs[-2]:
                print("bumper gauche et droit ")
                self.env.robot.reinit()
                for i in range(24):
                    self.obs, self.rew, self.done, self.info = self.mouvement(-2, 2, 1)
        
                # si element a gauche, tourner a droite
            elif self.obs[-3]:
                print("bumper gauche")
                self.env.robot.reinit()
                for i in range(24):
                    self.obs, self.rew, self.done, self.info = self.mouvement(-2, 2, 1)
        
        
                # si element a droite, tourner a gauche
            elif self.obs[-2]:
                print("bumper droit")
                self.env.robot.reinit()
                for i in range(24):
                    self.obs, self.rew, self.done, self.info = self.mouvement(2, -2, 1)
        
        now = time.time()
        print("%d timesteps took %f seconds" % (self.i, now - self.then))
        print("objectif diametre: " + str(self.diametre_goal))
        input("Press Enter to continue...")
        self.env.close()
        
    def reactifBumper(self):
        while True:
            # si aucun element devant, avancer
            if not (self.obs[-3] or self.obs[-2]):
                self.obs, self.rew, self.done, self.info = self.mouvement(1, 1, 1)
            else:
                print("bumper gauche et droit ")
                direction = random.uniform(0, 1)
                angle = random.randrange(24)
                self.env.robot.reinit()
                if direction <= 0.5:
                    for i in range(angle):
                        self.obs, self.rew, self.done, self.info = self.mouvement(2, -2, 1)
                else:
                    for i in range(angle):
                        self.obs, self.rew, self.done, self.info = self.mouvement(-2, 2, 1)
        now = time.time()
        print("%d timesteps took %f seconds" % (self.i, now - self.then))
        print("objectif diametre: " + str(self.diametre_goal))
        input("Press Enter to continue...")
        self.env.close()

    def reactifLaser(self):
        while True:
            for i in range (10):
                if self.obs[i] < 60:
                    if i <= 4:
                        print("element a gauche")
                        self.env.robot.reinit()
                        for i in range(24):
                            self.obs, self.rew, self.done, self.info = self.mouvement(-2, 2, 1)
                    else:
                        print("element a gauche")
                        self.env.robot.reinit()
                        for i in range(24):
                            self.obs, self.rew, self.done, self.info = self.mouvement(-2, 2, 1)  
                    break
                elif i == 9:
                    self.obs, self.rew, self.done, self.info = self.mouvement(1, 1, 1)
        now = time.time()
        print("%d timesteps took %f seconds" % (self.i, now - self.then))
        print("objectif diametre: " + str(self.diametre_goal))
        input("Press Enter to continue...")
        self.env.close()


if __name__=="__main__":
    env = 'kitchen-v1'
    sleep_time = 0.01
    display = True
    simEnv = SimEnv(env, sleep_time, display)
    simEnv.reactifLaser()