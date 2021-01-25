#!/usr/bin/python
# -*- coding: utf-8 -*-

""" Alexandre Coninx
    ISIR CNRS/UPMC
    05/06/2019
""" 

import gym, gym_fastsim
import time
import random


display= True

env = gym.make('kitchen-v0')
env.reset()
action=[10,11]

if(display):
	env.enable_display()

then = time.time()
i=0
o,r,eo,info=env.step(action)
print("Step %d Obs=%s  reward=%f  dist. to objective=%f  robot position=%s  End of ep=%s" % (i, str(o), r, info["dist_obj"], str(info["robot_pos"]), str(eo)))
print(env.robot.get_collision())

while info["dist_obj"]>10 :
        
    env.render()
    o,r,eo,info=env.step([random.randint(-2,5),random.randint(-2,5)])

    if env.robot.get_collision()==True:
        env.robot.reinit()

        o,r,eo,info=env.step([-1, 1 ])
        o,r,eo,info=env.step([5,5 ])

    print("Step %d Obs=%s  reward=%f  dist. to objective=%f  robot position=%s  End of ep=%s" % (i, str(o), r, info["dist_obj"], str(info["robot_pos"]), str(eo)))
    
    if(display):
        time.sleep(0.01)
    if eo:
        break
    i+=1


now = time.time()

print("%d timesteps took %f seconds" % (i, now - then))

env.close()
