import os
import time
import csv
import argparse
import gym
from time import sleep
from iRobot_gym.envs import SimpleNavEnv
from controllers.follow_wall import Follow_wallController
from controllers.forward import ForwardController
from controllers.rulebased import RuleBasedController
from controllers.braitenberg import BraitenbergController
from controllers.novelty_ctr import NoveltyController


ListePosition = []
TimeSampling = []


class SimEnv():

    def __init__(self, env, ctr, file_name, sleep_time):
        self.env = gym.make(env+str('-v0'))
        self.env.reset()
        self.sleep_time = sleep_time
        self.obs, self.rew, self.done, self.info = self.env.step([1, 1])

        # initialize controllers
        if ctr == "forward":
            self.controller = ForwardController(self.env, verbose=False)
        elif ctr == "wall":
            self.controller = Follow_wallController(self.env, verbose=False)
        elif ctr == "rule":
            self.controller = RuleBasedController(self.env, verbose=False)
        elif ctr == "brait":
            self.controller = BraitenbergController(self.env, verbose=False)
        elif ctr == "novelty":
            self.controller = NoveltyController(self.env, file_name)

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
        # start timers
        then = time.time()
        t1 = then
        self.i = 0

        while not self.done:
            try:
                if self.i % 1 == 0:
                    command = self.controller.get_command()
                    self.obs, self.rew, self.done, self.info = self.mouvement(
                        command)
                    x, y, z, roll, pitch, yaw = self.info['pose']
                    ListePosition.append(
                        [self.i, x, y, z, roll, pitch, yaw, self.info["dist_obj"], self.obs])
                    t2 = time.time()
                    if t2 - t1 >= 1:
                        t1 = t2
                        TimeSampling.append([x, y])
                    self.controller.reset()

                print(self.i, end='\r')
                self.i += 1

            except KeyboardInterrupt:
                print('\nAll done')
                break

        print("number of steps divided:", self.i)
        print("time:", self.info['time'])
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


def save_time_sampling(name, controller):
    base_path = os.path.dirname(os.path.abspath(__file__))
    path = f'{base_path}/../time_sampling/{name}/bullet{controller}_'
    i = 1
    if os.path.exists(path+str(i)+".csv"):
        while os.path.exists(path+str(i)+".csv"):
            i += 1
    with open(path+str(i)+".csv", 'w', newline='') as file:
        writer = csv.writer(file)
        print("\ndata saved as ", file)
        writer.writerow(["x", "y"])
        writer.writerows(TimeSampling)


if __name__ == "__main__":

    parser = argparse.ArgumentParser(
        description='Launch pybullet simulation run.')
    parser.add_argument('--env', type=str, default="maze_hard",
                        help='environnement, kitchen, maze_hard, race_track')
    # "forward", "wall", "rule", "brait", "novelty"
    parser.add_argument('--ctr', type=str, default="rule",
                        help='controller')
    parser.add_argument('--sleep_time', type=int, default=0.001,
                        help='sleeping time between each step')
    parser.add_argument('--file_name', type=str,
                        default='NoveltyFitness/3/maze_nsfit3-gen59-p0', help='file name for')

    args = parser.parse_args()
    env = args.env
    ctr = args.ctr
    file_name = args.file_name
    sleep_time = args.sleep_time

    simEnv = SimEnv(env, ctr, file_name, sleep_time)
    simEnv.start()
    save_result(env, ctr)
    # save_time_sampling(env, ctr)
