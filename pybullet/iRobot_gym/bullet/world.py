"""In this file we import all our parameters and initialize all our objects (walls, robot, goal).
 We also update the last information
 (position, acceleration, velocity, distance to the goal and time)."""

import os
import random
import math
from dataclasses import dataclass
import gym

import numpy as np
import pybullet_data
import pybullet as p

from iRobot_gym.bullet import util
from iRobot_gym.bullet.configs import GoalConfig, SimulationConfig, PhysicsConfig


class World:
    """In this Class we import all our parameters and initialize all our objects and update the stat of the agent.

    Attributes:
        _time: A float representing the time of the simulation.
        _agents: An Agent object (the robot).
        _state: A dictionary that will contain the position, acceleration, velocity, distance to objectif of the agent.
    """
    @dataclass
    class Config:
        name: str
        sdf: str
        scale: float
        goal_config: GoalConfig
        simulation_config: SimulationConfig
        physics_config: PhysicsConfig

    def __init__(self, config: Config, agents):
        self._config = config
        self._time = 0.0
        self._agents = agents
        self._state = dict([(a.id, {}) for a in agents])

    def init(self):
        """Init the simulation by loading the scene, the goal, the agent, the physics and the camera"""

        if self._config.simulation_config.GUI:
            p.connect(p.GUI)  # render True
        else:
            p.connect(p.DIRECT)  # render False

        self._load_scene(self._config.sdf)
        self._load_goal()
        p.setTimeStep(self._config.simulation_config.time_step)

        if not (self._config.simulation_config.GUI and self._config.simulation_config.following_camera):
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

        p.setGravity(0, 0, self._config.physics_config.gravity)

        p.resetDebugVisualizerCamera(
            cameraDistance=0.75*self._config.scale, cameraYaw=0, cameraPitch=-89.999,
            cameraTargetPosition=[self._config.scale/2, self._config.scale/2, 0])

    def _load_scene(self, sdf_file: str):
        p.loadURDF(sdf_file, globalScaling=self._config.scale)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.loadURDF('plane.urdf')

    def _load_goal(self):
        base_path = os.path.dirname(os.path.abspath(__file__))
        p.loadURDF(f'{base_path}/../../models/scenes/goal.urdf',
                   self._config.goal_config.goal_position, globalScaling=self._config.goal_config.goal_size)

    @classmethod
    def _get_starting_position(cls, agent):
        return agent.starting_position, agent.starting_orientation

    def _update_info(self, agent):
        goal_pos = self._config.goal_config.goal_position

        pose = util.get_pose(id=agent.vehicle_id)
        if pose is None:
            self._state[agent.id]['pose'] = np.append((0, 0, 0), (0, 0, 0))
        else:
            self._state[agent.id]['pose'] = pose

        velocity = util.get_velocity(id=agent.vehicle_id)
        if 'velocity' in self._state[agent.id]:
            previous_velocity = self._state[agent.id]['velocity']
            self._state[agent.id]['acceleration'] = (
                velocity - previous_velocity) / self._config.simulation_config.time_step
        else:
            self._state[agent.id]['acceleration'] = velocity / \
                self._config.simulation_config.time_step
        self._state[agent.id]['velocity'] = velocity

        self._state[agent.id]['time'] = self._time
        self._state[agent.id]['dist_obj'] = math.sqrt(
            (pose[0]-goal_pos[0])**2+(pose[1]-goal_pos[1])**2)

    def update(self, agent_id: str, width=640, height=480):
        """moves the simulation forward: incremente the time,
        step the simulation and update the following camera if needed.

        Args:
            agent_id: String, the agent id.
            width: A int, the width of the camera.
            height: A int, the height of the camera.
        """
        p.stepSimulation()
        self._time += self._config.simulation_config.time_step
        if self._config.simulation_config.GUI and self._config.simulation_config.following_camera:
            agent = list(filter(lambda a: a.id == agent_id, self._agents))
            util.follow_agent(agent=agent[0], width=width, height=height)

    def state(self):
        """Update state of the agents"""
        for agent in self._agents:
            self._update_info(agent=agent)
        return self._state

    def render(self, agent_id, width=640, height=480):
        """render the following camera.

        Args:
            agent_id: String, the agent id.
            width: A int, the width of the camera.
            height: A int, the height of the camera.
        """
        agent = list(filter(lambda a: a.id == agent_id, self._agents))
        return util.follow_agent(agent=agent[0], width=width, height=height)

    def seed(self, seed=None):
        """Generate a seed if needed"""
        if self is None:
            seed = 0
        np.random.seed(seed)
        random.seed(seed)

    def reset(self):
        """reset the class"""
        p.setTimeStep(self._config.simulation_config.time_step)
        p.stepSimulation()
        self._time = 0.0
        self._state = dict([(a.id, {}) for a in self._agents])

    def space(self):
        """Descibe the format of valid action for gym."""
        return gym.spaces.Dict({'time': gym.spaces.Box(low=0, high=math.inf, shape=(1,))})
