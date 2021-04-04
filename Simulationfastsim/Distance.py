#!/usr/bin/python
# -*- coding: utf-8 -*-
import gym, gym_fastsim
import time
import random
import sys
import numpy as np

def mouvement(l, r, n):
    for k in range(n):
        obs, rew, done, info = env.step([l,r])
        print("Valeur de obs :" + str(obs))
        print("Valeur de rew :" + str(rew))
        print("Valeur de done :" + str(done))
        print("Valeur de info :" + str(info))
        oldDist = info["dist_obj"]
        if(display):
            time.sleep(time_sleep)
        if done:
            break
        env.render()
        global i
        i+= 1
        # print("Step %d Obs=%s reward=%f  dist. to objective=%f  robot position=%s" % (i, str([round(num,1) for num in o]),r, info["dist_obj"], str(info["robot_pos"]) ) , end="\r" )
        return obs, rew, done, info

display = True
time_sleep= 0.01

env = gym.make('kitchen-v0')
env.reset()

if(display):
	env.enable_display()

then = time.time()
i = 0 
obs, rew, done, info = env.step([0,0])
diametre_goal= env.goal.get_diam()

dist = np.inf

while dist > diametre_goal :

    # si aucun element devant, avancer
    if not (obs[2] or obs[3]):
        obs, rew, done, info = mouvement(1, 1, 1)
        newDist = info["dist_obj"]
        # si on s'eloigne, tourner a droite
        if newDist > dist:
            for i in range(24):
                mouvement(-2, 2, 1)
        dist = newDist
    
    # si element devant, tourner a droite
    elif obs[2] and obs[3]:
        print("bumper gauche et droit ")
        env.robot.reinit()
        for i in range(24):
            obs, rew, done, info = mouvement(-2, 2, 1)

        # si element a gauche, tourner a droite
    elif obs[2]:
        print("bumbpr gauche")
        env.robot.reinit()
        for i in range(24):
            obs, rew, done, info = mouvement(-2, 2, 1)


        # si element a droite, tourner a gauche
    elif obs[3]:
        print("bumper droit")
        env.robot.reinit()
        for i in range(24):
            obs, rew, done, info = mouvement(2, -2, 1)

    
    
now = time.time()

print("%d timesteps took %f seconds" % (i, now - then))
print("objectif diametre: " + str(diametre_goal))

input("Press Enter to continue...")
env.close()