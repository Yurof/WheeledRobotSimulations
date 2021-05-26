import os
import subprocess
import time
import signal
import numpy as np
import gym
from gym import error, spaces
from gym import utils
from gym.utils import seeding
import math
import random
import logging

import pyfastsim as fs

logger = logging.getLogger(__name__)

# default_env = "assets/LS_maze_hard.xml"
default_env = "assets/example.xml"

# default is True in libfastsim... but false in the fastsim sferes2 module -> stick (haha) with that
sticky_walls = False


def sqdist(x, y):
    return (x[0]-y[0])**2+(x[1]-y[1])**2


def dist(x, y):
    return math.sqrt(sqdist(x, y))


def reward_binary_goal_based(navenv):
    """ Reward of 1 is given when close enough to the goal. """
    curpos = navenv.current_pos
    if (dist(curpos, navenv.goalPos) <= navenv.goalRadius):
        return 1.
    else:
        return 0.


def reward_minus_energy(navenv):
    """ Reward = minus sum of absolute values of motor orders"""
    r = -(np.abs(navenv.v1_motor_order) + np.abs(navenv.v2_motor_order))
    return r


def reward_displacement(navenv):
    """ Reward = distance to previous position"""
    r = dist(navenv.current_pos, navenv.old_pos)
    return r


def no_reward(navenv):
    """ No reward"""
    return 0.


reward_functions = {"binary_goalbased": reward_binary_goal_based,
                    "minimize_energy": reward_minus_energy,
                    "displacement": reward_displacement,
                    "none": no_reward,
                    None: no_reward}


class SimpleNavEnv(gym.Env):
    def __init__(self, xml_env, reward_func="binary_goalbased", render=False, light_sensor_range=200., light_sensor_mode="realistic"):
        # Fastsim setup
        # XML files typically contain relative names (for map) wrt their own path. Make that work
        xml_dir, xml_file = os.path.split(xml_env)
        if(xml_dir == ""):
            xml_dir = "./"
        oldcwd = os.getcwd()
        os.chdir(xml_dir)
        settings = fs.Settings(xml_file)
        os.chdir(oldcwd)

        self.map = settings.map()
        self.robot = settings.robot()

        if(render):
            self.display = fs.Display(self.map, self.robot)
        else:
            self.display = None

        self.maxVel = 4  # Same as in the C++ sferes2 experiment

        # Lasers
        lasers = self.robot.get_lasers()
        n_lasers = len(lasers)
        if(n_lasers > 0):
            # Assume at least 1 laser ranger
            self.maxSensorRange = lasers[0].get_range()
        else:
            self.maxSensorRange = 0.

        # Light sensors
        self.ls_mode = light_sensor_mode
        lightsensors = self.robot.get_light_sensors()
        n_lightsensors = len(lightsensors)
        if(n_lightsensors > 0):
            self.maxLightSensorRange = light_sensor_range  # Assume at least 1 laser ranger
        else:
            self.maxLightSensorRange = 0.

        # State
        self.initPos = self.get_robot_pos()
        self.current_pos = self.get_robot_pos()
        self.old_pos = self.get_robot_pos()

        self.v1_motor_order = 0.
        self.v2_motor_order = 0.

        self.goal = self.map.get_goals()[0]  # Assume 1 goal
        self.goalPos = [self.goal.get_x(), self.goal.get_y()]
        self.goalRadius = self.goal.get_diam()/2.

        self.observation_space = spaces.Box(low=np.array([0.]*n_lasers + [0.]*2 + [0. if (self.ls_mode == "realistic") else -1]*n_lightsensors), high=np.array(
            [self.maxSensorRange]*n_lasers + [1. if (self.ls_mode == "realistic") else self.maxLightSensorRange]*2 + [1.]*n_lightsensors, dtype=np.float32))
        self.action_space = spaces.Box(
            low=-self.maxVel, high=self.maxVel, shape=(2,), dtype=np.float32)

        # Reward
        if(reward_func not in reward_functions):
            raise RuntimeError("Unknown reward '%s'" % str(reward_func))
        else:
            self.reward_func = reward_functions[reward_func]

    def enable_display(self):
        if not self.display:
            self.display = fs.Display(self.map, self.robot)
            self.display.update()

    def disable_display(self):
        if self.display:
            del self.display
            self.display = None

    def get_robot_pos(self):
        pos = self.robot.get_pos()
        return [pos.x(), pos.y(), pos.theta()]

    def get_laserranges(self):
        out = list()
        for l in self.robot.get_lasers():
            # print("GET DIST :" + str(l.get_dist()))
            # print("GET ANGLE :" + str(l.get_angle()))
            # print("GET RANGE :" + str(l.get_range()))
            # print("GET GAP DIST :" + str(l.get_gap_dist()))
            # print("GET GAP ANGLE :" + str(l.get_gap_angle()))
            r = l.get_dist()
            if r < 0:
                out.append(self.maxSensorRange)
            else:
                out.append(np.clip(r, 0., self.maxSensorRange))
        return out

    def get_lightsensors(self):
        out = list()
        for ls in self.robot.get_light_sensors():
            r = ls.get_distance()
            if(self.ls_mode == "realistic"):
                # r is -1 if no light, dist if light detected
                # We want the output to be a "realistic" light sensor :
                # - 0 if no light (no target in field or out of range)
                # - (1-d/maxRange)**2 if light detectted
                # Out of range, or not in angular range
                if((r < 0) or (r > self.maxLightSensorRange)):
                    out.append(0.)
                else:
                    out.append((1. - r/self.maxLightSensorRange)**2)
            elif(self.ls_mode == "raw"):
                out.append(r)
            else:
                raise RuntimeError("Unknown LS mode: %s" % self.ls_mode)

        return out

    def get_bumpers(self):
        return [float(self.robot.get_left_bumper()), float(self.robot.get_right_bumper())]

    def get_all_sensors(self):
        # return self.get_laserranges() + self.get_bumpers() + self.get_lightsensors()
        return self.get_laserranges()

    def step(self, action):
        # Action is: [leftWheelVel, rightWheelVel]
        [v1, v2] = action

        self.v1_motor_order = np.clip(v1, -self.maxVel, self.maxVel)
        self.v2_motor_order = np.clip(v2, -self.maxVel, self.maxVel)

        self.robot.move(self.v1_motor_order,
                        self.v2_motor_order, self.map, sticky_walls)

        sensors = self.get_all_sensors()
        reward = self._get_reward()

        self.old_pos = self.current_pos
        self.current_pos = self.get_robot_pos()

        # if(sqdist(p,self.roldpos)<0.001**2):
        #	self.still=self.still+1
        # else:
        #	self.still=0
        # self.roldpos=p
        #episode_over = self.still>=self.still_limit

        dist_obj = dist(self.current_pos, self.goalPos)
        episode_over = dist_obj <= self.goal.get_diam()

        return sensors, reward, episode_over, {"dist_obj": dist_obj, "robot_pos": self.current_pos}

    def _get_reward(self):
        return self.reward_func(self)  # Use reward extraction function

    def reset(self):
        p = fs.Posture(*self.initPos)
        self.robot.set_pos(p)
        self.current_pos = self.get_robot_pos()
        self.v1_motor_order = 0.
        self.v2_motor_order = 0.
        return self.get_all_sensors()

    def render(self, mode='human', close=False):
        if self.display:
            self.display.update()
        pass

    def close(self):
        self.disable_display()
        del self.robot
        del self.map
        pass

    def get_map_size(self):
        return self.map.get_real_h()
