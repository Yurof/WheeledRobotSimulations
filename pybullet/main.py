from controllers.follow_wall import Follow_wallController
from controllers.forward import ForwardController
from controllers.rulebased import RuleBasedController
from controllers.braitenberg import BraitenbergController


import time
from time import sleep
import gym
from iRobot_gym.envs import SimpleNavEnv
import os
import csv


ListePosition = []


class SimEnv():

    def __init__(self, env, sleep_time):
        self.env = gym.make(env+str('-v0'))
        self.env.reset()
        self.sleep_time = sleep_time
        self.obs, self.rew, self.done, self.info = self.env.step([0, 0])

    def mouvement(self, c, n=1):
        for _ in range(n):
            obs, rew, done, info = self.env.step(c)

            #print("Valeur de obs :" + str(obs))
            #print("Valeur de rew :" + str(rew))
            #print("Valeur de done :" + str(done))
            #print("Valeur de info :" + str(info))
            if self.done:
                break
            time.sleep(self.sleep_time)

        return obs, rew, done, info

    def start(self):

        # initialize controllers
        forward = ForwardController(self.env, verbose=False)
        wall = Follow_wallController(self.env, verbose=False)
        rule = RuleBasedController(self.env, verbose=False)
        brait = BraitenbergController(self.env, verbose=False)
        self.controller = brait

        # start timers
        then = time.time()
        self.i = 0

        while not self.done:
            try:
                if self.i % 1 == 0:
                    command = self.controller.get_command()
                    self.obs, self.rew, self.done, self.info = self.mouvement(
                        command)
                    x, y, z, roll, pitch, yaw = self.info['pose']
                    ListePosition.append(
                        [self.i, x, y, z, roll, pitch, yaw, self.info["progress"], self.obs['lidar']])
                    self.controller.reset()

                print(self.i, end='\r')
                self.i += 1
            except KeyboardInterrupt:
                print('All done')
                break
        now = time.time()
        print("%d timesteps took %f seconds" % (self.i, now - then))
        self.env.logfile()
        self.env.close()


def save_result(name, controller):
    base_path = os.path.dirname(os.path.abspath(__file__))
    path = f'{base_path}/../results/{name}/bullet_{controller}_'
    i = 1
    if os.path.exists(path+str(i)+".csv"):
        while os.path.exists(path+str(i)+".csv"):
            i += 1
    with open(path+str(i)+".csv", 'w', newline='') as file:
        writer = csv.writer(file)
        print("\ndata saved as ", file)
        writer.writerow(["steps", "x", "y", "z", "roll", "pitch", "yaw",
                         "distance_to_obj", "lidar"])
        writer.writerows(ListePosition)


if __name__ == "__main__":
    env1 = 'kitchen'
    env2 = 'maze_hard'
    env3 = 'race_track'
    sleep_time = 0.00001
    simEnv = SimEnv(env3, sleep_time)
    simEnv.start()
    save_result(env3, 'brait')
