"File to test the robot in a empty map"
import os
import time
import gym
from iRobot_gym.envs import SimpleNavEnv
import numpy as np


class SimEnv():

    def __init__(self, env):
        self._env = gym.make(env+str('-v0'))
        self._env.reset()
        self._i = 0
        self.obs, self.rew, self.done, self.info = self._env.step([0, 0])

    def mouvement(self, c, n=1):
        for _ in range(n):
            obs, rew, done, info = self._env.step(c)
            time.sleep(sleep_time)
            # print("Value of obs :" + str(obs))
            # print("Value of rew :" + str(rew))
            # print("Value of done :" + str(done))

            self._i += 1
            if done:
                break
        return obs, rew, done, info

    def forward(self):
        mean_l = []
        while not self.done and self.info['pose'][0] < 100:
            try:
                self.obs, self.rew, self.done, self.info = self.mouvement([
                                                                          1, 1])
                x, y, z, roll, pitch, yaw = self.info['pose']
                print(self._i, end='\r')
                mean_l.append(self.info['velocity'][0])

            except KeyboardInterrupt:
                print(' The simulation was forcibly stopped.')
                break

        print("number of steps:", self._i)
        print("real time is:", self.info['time'])
        print('mean velocity', np.mean(mean_l))
        self._env.close()

    def rotation(self):

        while not self.done:
            try:
                self.obs, self.rew, self.done, self.info = self.mouvement([
                                                                          1, -1])
                x, y, z, roll, pitch, yaw = self.info['pose']
                print(self._i, yaw)

            except KeyboardInterrupt:
                print(' The simulation was forcibly stopped.')
                break

        print("number of steps:", self._i)
        print("real time is:", self.info['time'])
        self._env.close()


if __name__ == "__main__":
    sleep_time = 0.0001
    simEnv = SimEnv('blank')
    simEnv.forward()
    # simEnv.rotation()
