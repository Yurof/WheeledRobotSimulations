import math
import random
from dataclasses import dataclass
from typing import Dict, Any, List

import gym
import os
import numpy as np
import pybullet as p
from PIL import Image
from gym import logger

from iRobot_gym.bullet import util
from iRobot_gym.bullet.configs import GoalConfig, AgentSpec, SimulationConfig, PhysicsConfig
from iRobot_gym.core import world
from iRobot_gym.core.agent import Agent
from iRobot_gym.core.definitions import Pose


class World(world.World):

    @dataclass
    class Config:
        name: str
        sdf: str
        goal_config: GoalConfig
        simulation_config: SimulationConfig
        physics_config: PhysicsConfig

    def __init__(self, config: Config, agents: List[Agent]):
        self._config = config
        self._map_id = None
        self._time = 0.0
        self._agents = agents
        self._state = dict([(a.id, {}) for a in agents])
        self._objects = {}

    def init(self) -> None:
        if self._config.simulation_config.rendering:
            p.connect(p.GUI)  # render True
        else:
            p.connect(p.DIRECT)  # render False

        self._load_scene(self._config.sdf)
        self._load_goal()
        p.setTimeStep(self._config.simulation_config.time_step)
        p.setGravity(0, 0, self._config.physics_config.gravity)
        p.resetDebugVisualizerCamera(
            cameraDistance=4.6, cameraYaw=0, cameraPitch=-89.9, cameraTargetPosition=[2.5, 2, 0])

    def reset(self):
        p.setTimeStep(self._config.simulation_config.time_step)
        p.setGravity(0, 0, self._config.physics_config.gravity)
        p.stepSimulation()
        self._time = 0.0
        self._state = dict([(a.id, {}) for a in self._agents])

    def _load_scene(self, sdf_file: str):
        ids = p.loadSDF(sdf_file)
        objects = dict([(p.getBodyInfo(i)[1].decode('ascii'), i) for i in ids])
        self._objects = objects

    def _load_goal(self):
        base_path = os.path.dirname(os.path.abspath(__file__))
        print("goal position", self._config.goal_config.goal_position)
        p.loadURDF(f'{base_path}/../../models/scenes/goal.urdf',
                   self._config.goal_config.goal_position, globalScaling=self._config.goal_config.goal_size)

    def get_starting_position(self, agent: Agent) -> Pose:
        return self._agents[0].starting_position,  self._agents[0].starting_orientation

    def update(self):
        p.stepSimulation()
        self._time += self._config.simulation_config.time_step

    def state(self) -> Dict[str, Any]:
        for agent in self._agents:
            self._update_race_info(agent=agent)
        return self._state

    def space(self) -> gym.Space:
        return gym.spaces.Dict({
            'time': gym.spaces.Box(low=0, high=math.inf, shape=(1,))
        })

    def _update_race_info(self, agent):
        goal_pos = self._config.goal_config.goal_position
        pose = util.get_pose(id=agent.vehicle_id)
        if pose is None:
            logger.warn('Could not obtain pose.')
            self._state[agent.id]['pose'] = np.append((0, 0, 0), (0, 0, 0))
        else:
            self._state[agent.id]['pose'] = pose
        collision_with_wall = False
        self._state[agent.id]['wall_collision'] = collision_with_wall

        velocity = util.get_velocity(id=agent.vehicle_id)
        if 'velocity' in self._state[agent.id]:
            previous_velocity = self._state[agent.id]['velocity']
            self._state[agent.id]['acceleration'] = (
                velocity - previous_velocity) / self._config.simulation_config.time_step
        else:
            self._state[agent.id]['acceleration'] = velocity / \
                self._config.simulation_config.time_step

        pose = self._state[agent.id]['pose']
        self._state[agent.id]['velocity'] = velocity
        self._state[agent.id]['progress'] = math.sqrt(
            (pose[0]-goal_pos[0])**2+(pose[1]-goal_pos[1])**2)
        self._state[agent.id]['time'] = self._time

    def render(self, agent_id: str, width=640, height=480) -> np.ndarray:
        agent = list(filter(lambda a: a.id == agent_id, self._agents))
        assert len(agent) == 1
        agent = agent[0]
        return util.follow_agent(agent=agent, width=width, height=height)

    def seed(self, seed: int = None):
        if self is None:
            seed = 0
        np.random.seed(seed)
        random.seed(seed)
        p.ran
