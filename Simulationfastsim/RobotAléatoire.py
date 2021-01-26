#!/usr/bin/python
# -*- coding: utf-8 -*-
import gym, gym_fastsim
import time
import random
import sys


display = True
time_sleep= 0.01

env = gym.make('kitchen-v0')
env.reset()

if(display):
	env.enable_display()

then = time.time()
i = 0 
o, r, eo, info = env.step([0,0])
diametre_goal= env.goal.get_diam()

def mouvement(l, r, n):
    for k in range(n):
        o, r, eo, info = env.step([l,r])
        if(display):
            time.sleep(time_sleep)
        if eo:
            break
        env.render()
        global i
        i+= 1
        print("Step %d Obs=%s reward=%f  dist. to objective=%f  robot position=%s" % (i, str([round(num,1) for num in o]),r, info["dist_obj"], str(info["robot_pos"]) ) , end="\r" )


while info["dist_obj"]>diametre_goal :
    if env.robot.get_left_bumper() and env.robot.get_right_bumper():
        #print("bumber gauche et droit ")
        env.robot.reinit()
        mouvement(-2, 2, 20)

    elif env.robot.get_left_bumper():
        #print("bumber gauche")
        env.robot.reinit()
        mouvement(-2, 2, 1)

    elif env.robot.get_right_bumper():
        #print("bumber droit")
        env.robot.reinit()
        mouvement(2, -2, 1)

    else:
        mouvement(random.randint(-2,5),random.randint(-2,5),1)


now = time.time()

print("%d timesteps took %f seconds" % (i, now - then))

env.close()
