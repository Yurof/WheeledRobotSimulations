""" load .yml configuration files"""
import os
from dataclasses import dataclass
from typing import Any

from iRobot_gym.bullet.actuators import Motor
from iRobot_gym.bullet.configs import VehicleConfig, ScenarioSpec, WorldSpec, VehicleSpec
from iRobot_gym.bullet.sensors import Laser, FixedTimestepSensor
from iRobot_gym.bullet.vehicle import IRobot
from iRobot_gym.bullet.world import World


base_path = os.path.dirname(os.path.abspath(__file__))


class Agent:

    def __init__(self, id: str, vehicle, task_name, task_param, starting_position, starting_orientation):
        self._id = id
        self._vehicle = vehicle
        self._task_name = task_name
        self._task_param = task_param
        self._starting_position = starting_position
        self._starting_orientation = starting_orientation

    @property
    def id(self) -> str:
        return self._id

    @property
    def task_name(self) -> str:
        return self._task_name

    @property
    def task_param(self) -> str:
        return self._task_param

    @property
    def starting_position(self) -> str:
        return self._starting_position

    @property
    def starting_orientation(self) -> str:
        return self._starting_orientation

    @property
    def vehicle_id(self) -> Any:
        return self._vehicle.id

    def step(self, action):
        observation = self._vehicle.observe()
        self._vehicle.control(action)
        return observation, {}

    def reset(self, pose):
        self._vehicle.reset(pose=pose)
        observation = self._vehicle.observe()
        return observation


def load_vehicle(spec: VehicleSpec):
    config_file_path = f'{base_path}/../../configuration/robots/{spec.name}.yml'
    if not os.path.exists(config_file_path):
        raise NotImplementedError(
            f'No vehicle with name {spec.name} implemented.')

    config = VehicleConfig()
    config.load(config_file_path)
    config.urdf_file = f'{os.path.dirname(config_file_path)}/{config.urdf_file}'
    requested_sensors = set(spec.sensors)
    available_sensors = set([sensor.name for sensor in config.sensors])

    if not requested_sensors.issubset(available_sensors):
        raise NotImplementedError(
            f'Sensors {requested_sensors - available_sensors} not available.')
    sensors = list(
        filter(lambda s: s.name in requested_sensors, config.sensors))
    sensors = [FixedTimestepSensor(sensor=Laser(name=c.name, type=c.type, config=Laser.Config(**c.params)), frequency=c.frequency, time_step=0.01) for c in
               sensors]
    actuators = [Motor(name=c.name, config=Motor.Config(**c.params))
                 for c in config.actuators]
    car_config = IRobot.Config(urdf_file=config.urdf_file)
    vehicle = IRobot(sensors=sensors, actuators=actuators, config=car_config)
    return vehicle


def load_world(spec: WorldSpec, agents):
    config_file_path = f'{base_path}/../../configuration/scenarios/{spec.name}.yml'
    config = ScenarioSpec()
    config.load(config_file_path)

    sdf_path = f'{base_path}/../../models/scenes/{spec.name}/{spec.sdf}'

    world_config = World.Config(
        name=spec.name,
        sdf=sdf_path,
        scale=spec.scale,
        goal_config=spec.goal,
        simulation_config=spec.simulation,
        physics_config=spec.physics
    )

    return World(config=world_config, agents=agents)


@dataclass
class SimpleNavScenario:
    world: World
    agent: Agent

    @staticmethod
    def from_spec(path, rendering=False):
        spec = ScenarioSpec()
        spec.load(path)
        if rendering:
            spec.world.rendering = rendering
        agent_spec = spec.agents
        agent = Agent(id=agent_spec.id, vehicle=load_vehicle(agent_spec.vehicle), task_name=agent_spec.task.task_name,
                      task_param=agent_spec.task.params, starting_position=agent_spec.starting_position, starting_orientation=agent_spec.starting_orientation)
        return SimpleNavScenario(world=load_world(spec.world, agents=[agent]), agent=agent)
