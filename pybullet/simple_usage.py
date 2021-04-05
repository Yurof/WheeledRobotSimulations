import gym
from iRobot_gym.envs import SimpleNavEnv
import matplotlib.pyplot as plt
import os
import numpy as np

env = gym.make('Kitchen-v0')
env = gym.make('Maze_hard-v0')

done = False

for i_episode in range(2):
    Liste = []
    observation = env.reset()
    for t in range(1000):
        env.render()
        # print(observation)
        action = env.action_space.sample()
        observation, reward, done, info = env.step(action)
        Liste.append(info['pose'][:2])
        # print(info['pose'][:2])
        print(i_episode, t, end="\r")
        if done:
            print("Episode finished after {} timesteps".format(t+1))
            break
    Liste = [i * 80 for i in Liste]
    #print("\n", Liste)
    plt.scatter(*zip(*Liste), s=5, label="test")

    # plt.xlim(0, 5)
    # plt.ylim(0, 5)
plt.legend()
plt.show()
env.close()
