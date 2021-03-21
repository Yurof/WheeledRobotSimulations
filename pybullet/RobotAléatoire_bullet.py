from time import sleep
import gym
from iRobot_gym.envs import SingleAgentRaceEnv
import random
from numpy import array

env = gym.make('Kitchen_Gui-v0')
#env = gym.make('Maze_hard_Gui-v0')

done = False
time_sleep= 1

obs = env.reset()

i = 0 
action = dict([('motor', array([0,0]))])
lidar_collision = 0.35
obs, rewards, done, states = env.step(action)

def mouvement(l, r, n):
    for k in range(n):
        obs, rewards, done, states = env.step(dict([('motor', array([l, r]))]))
        #print(l,r)
        #print(obs['lidar'][len(obs['lidar'])//2], end="\r")  
        sleep(time_sleep)
        image = env.render()
        # if states['wall_collision']==True:
        #     print("touche le mur")
        global i
        i+= 1
        print("Step  %d reward=%f robot position=%s progress=%f" % (i,rewards,  str(states["pose"][0:3]) ,states["progress"] ) , end="\r" )
                               
        return obs, rewards, done, states

while not done:
    if obs['lidar'][len(obs['lidar'])//2]< lidar_collision-0.1:
        #print("avant ")
        obs, rewards, done, states= mouvement(-2, 2, 2)

    elif obs['lidar'][0]< lidar_collision:
        #print("gauche? ")
        obs, rewards, done, states= mouvement(2, -2, 1)

    elif obs['lidar'][-1]< lidar_collision:
        #print("droite? ")
        obs, rewards, done, states= mouvement(-2, 2, 1)
    else:
        obs, rewards, done, states= mouvement(random.uniform(-2,5),random.uniform(-2,5),1)

print("Step  %d reward=%f robot position=%s progress=%f" % (i, rewards,  str(states["pose"][0:3]) ,states["progress"] ) , end="\r" )

env.close()
