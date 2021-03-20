from time import sleep
import gym
from iRobot_gym.envs import SingleAgentRaceEnv
import random
from numpy import array

env = gym.make('SingleAgentKitchen_Gui-v0')
#env = gym.make('SingleAgentMaze_hard_Gui-v0')

done = False

obs = env.reset()
t = 0
while not done:
    action = env.action_space.sample()
    #print("action",action)
    obs, rewards, done, states = env.step(action)
    sleep(0.01)
    print("Step  %d reward=%f robot position=%s dist_obj=%f" % (t,rewards,  str(states["pose"][0:3]) ,states["progress"] ) , end="\r" )
    if t % 1 == 0:
        # Currently, two rendering modes are available: 'birds_eye' and 'follow'
        # birds_eye: Follow an agent in birds eye perspective.
        # follow: Follow an agent in a 3rd person view.
        image = env.render()
    t+=1


env.close()