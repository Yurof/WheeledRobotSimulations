
# coding: utf-8

# pyMaze expriments

import gym
import numpy as np
import time
#import resource



# Fitness/evaluation function

default_max_step = 2000 # same as C++ sferes experiments


class EvaluationFunctor:
	def __init__(self, gym_env_name=None, gym_params={}, controller=None, controller_type=None, controller_params=None, output='-dist_obj',max_step=default_max_step, bd_function=None):
		global current_serial
		#print("Eval functor created")
		#Env
		#Controller
		self.out = output
		self.max_step = max_step
		self.evals = 0
		self.traj=None
		self.controller=controller
		self.controller_type=controller_type
		self.controller_params=controller_params
		if (gym_env_name is not None):
			self.set_env(gym_env_name, gym_params)
		else:
			self.env = None
		self.get_behavior_descriptor = bd_function
		
	def set_env(self, env_name, gym_params):
		self.env = gym.make(env_name, **gym_params)
		self.env.reset()
		self.env_name = self.env.unwrapped.spec.id
		if(self.controller is None): # Build controller
			if(self.controller_type is None):
				raise RuntimeError("Please either give a controller or specify controller type")
			self.controller = self.controller_type(self.env.observation_space.shape[0],self.env.action_space.shape[0], params=self.controller_params)
		else:
			if(self.controller_type is not None or self.controller_params is not None):
				print("WARNING: EvaluationFunctor built with both controller and controller_type/controller_params. controller_type/controller_params arguments  will be ignored")
