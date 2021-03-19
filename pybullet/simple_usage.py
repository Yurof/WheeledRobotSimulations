from time import sleep
import gym
from iRobot_gym.envs import SingleAgentRaceEnv
import random
from numpy import array

env = gym.make('SingleAgentKitchen_Gui-v0')
done = False
time_sleep= 0.1

obs = env.reset()

t = 0
action = dict([('motor', array([0,0]))])
lidar_collision = 0.5
obs, rewards, done, states = env.step(action)

def mouvement(l, r, n):
    for k in range(n):
        obs, rewards, done, states = env.step(dict([('motor', array([l, r]))]))
        #print(l,r)
        print(obs['lidar'][len(obs['lidar'])//2], end="\r")  
        sleep(time_sleep)
        image = env.render()                                 
        return obs, rewards, done, states

while not done:
    #action = env.action_space.sample()
    #print(t)
    #obs, rewards, done, states = env.step(action)
    #print(states['wall_collision'])
    if obs['lidar'][len(obs['lidar'])//2]< lidar_collision-0.1:
        print("avant ")
        obs, rewards, done, states= mouvement(-2, 2, 5)

    elif obs['lidar'][0]< lidar_collision:
        print("gauche? ")
        obs, rewards, done, states= mouvement(-2, 2, 1)

    elif obs['lidar'][-1]< lidar_collision:
        print("droite? ")
        obs, rewards, done, states= mouvement(2, -2, 1)
    else:
        obs, rewards, done, states= mouvement(random.uniform(-2,5),random.uniform(-2,5),1)

"""
sleep(100)
if t % 10 == 0:
    image = env.render()
t+=1
"""
env.close()

# Currently, there are two reset modes available: 'grid' and 'random'.
# Grid: Place agents on predefined starting position.
# Random: Random poses on the track.
# birds_eye: Follow an agent in birds eye perspective.
# follow: Follow an agent in a 3rd person view.

"""
    obs, rewards, done, states = env.step(action)
    if  states['wall_collision']==1:
        action = dict([('motor', array([random.uniform(-0.25,1)])), ('steering', array([random.uniform(-1,1)]))])
    else:
        action = dict([('motor', array([-0.61642513])), ('steering', array([-1]))])
    sleep(0.01)
"""

"""
    if  states['wall_collision']==False:
        action = dict([('motor', array([random.uniform(-0.25,0.7)])), ('steering', array([random.uniform(-1,0.5)]))])
        obs, rewards, done, states = env.step(action)
    else:
        print("contact",obs['lidar'][540])

        for k in  range(20):
            if t % 100 == 0:
                print("ici")
                action = dict([('motor', array([random.uniform(-0.25,1)])), ('steering', array([random.uniform(-1,1)]))])
            else:
                action = dict([('motor', array([-1])), ('steering', array([-0.25]))])
            obs, rewards, done, states = env.step(action)
        image = env.render(mode='follow')
    sleep(0.01)
    if t % 10 == 0:
        image = env.render(mode='follow')
    t+=1
"""