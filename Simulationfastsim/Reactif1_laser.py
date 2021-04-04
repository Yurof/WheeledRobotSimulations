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
        # print("Valeur de rew :" + str(rew))
        # print("Valeur de done :" + str(done))
        # print("Valeur de info :" + str(info))
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

# env = testenv.EvaluationFunctor('kitchen-v0')
env = gym.make('kitchen-v1')
env.reset()

if(display):
	env.enable_display()

then = time.time()
i = 0 
obs, rew, done, info = env.step([0,0])
diametre_goal= env.goal.get_diam()

dist = np.inf

while True:
    for i in range (10):
        if obs[i] < 60:
            if i <= 4:
                print("element a gauche")
                env.robot.reinit()
                for i in range(24):
                    obs, rew, done, info = mouvement(-2, 2, 1)
            else:
                print("element a gauche")
                env.robot.reinit()
                for i in range(24):
                    obs, rew, done, info = mouvement(-2, 2, 1)  
            break
        elif i == 9:
            obs, rew, done, info = mouvement(1, 1, 1)
                
    
    
    # # si aucun element devant, avancer
    # if obs[0] >= 60 and obs[1] >= 60:
    #     obs, rew, done, info = mouvement(1, 1, 1)
    # elif obs[0] < 60:
    #     print("element a droite")
    #     env.robot.reinit()
    #     for i in range(24):
    #         obs, rew, done, info = mouvement(2, -2, 1)
    # elif obs[1] < 60:
    #     print("element a gauche")
    #     env.robot.reinit()
    #     for i in range(24):
    #         obs, rew, done, info = mouvement(-2, 2, 1)     
    
    
now = time.time()

print("%d timesteps took %f seconds" % (i, now - then))
print("objectif diametre: " + str(diametre_goal))

input("Press Enter to continue...")
env.close()