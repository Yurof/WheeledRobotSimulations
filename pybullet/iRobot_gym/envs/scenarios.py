import os

from typing import List
from dataclasses import dataclass
from iRobot_gym import core
from iRobot_gym.tasks import get_task
from iRobot_gym.core import World, Agent
from iRobot_gym.bullet.actuators import Motor
from iRobot_gym.bullet.configs import VehicleConfig, ScenarioSpec, WorldSpec, VehicleSpec, TaskSpec
from iRobot_gym.bullet.sensors import Lidar, FixedTimestepSensor
from iRobot_gym.bullet.vehicle import IRobot
from iRobot_gym.bullet.world import World


base_path = os.path.dirname(os.path.abspath(__file__))


def load_vehicle(spec: VehicleSpec) -> core.Vehicle:
    config_file_path = f'{base_path}/../../models/{spec.name}/{spec.name}.yml'
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
    sensors = [FixedTimestepSensor(sensor=Lidar(name=c.name, type=c.type, config=Lidar.Config(**c.params)), frequency=c.frequency, time_step=0.01) for c in
               sensors]
    actuators = [Motor(name=c.name, config=Motor.Config(**c.params))
                 for c in config.actuators]
    car_config = IRobot.Config(urdf_file=config.urdf_file)
    vehicle = IRobot(sensors=sensors, actuators=actuators, config=car_config)
    return vehicle


def load_world(spec: WorldSpec, agents: List[Agent]) -> core.World:
    config_file_path = f'{base_path}/../../scenarios/{spec.name}.yml'
    config = ScenarioSpec()
    config.load(config_file_path)

    spec.simulation.rendering = spec.rendering
    sdf_path = f'{base_path}/../../models/scenes/{spec.name}/{spec.sdf}'

    world_config = World.Config(
        name=spec.name,
        sdf=sdf_path,
        goal_config=spec.goal,
        simulation_config=spec.simulation,
        physics_config=spec.physics
    )

    return World(config=world_config, agents=agents)


def task_from_spec(spec: TaskSpec):
    task = get_task(spec.task_name)
    return task(**spec.params)


@dataclass
class SimpleNavScenario:
    world: World
    agent: Agent

    @staticmethod
    def from_spec(path: str, rendering: bool = None) -> 'SimpleNavScenario':
        spec = ScenarioSpec()
        spec.load(path)
        if rendering:
            spec.world.rendering = rendering
        agent_spec = spec.agents
        agent = Agent(id=agent_spec.id, vehicle=load_vehicle(agent_spec.vehicle), task=task_from_spec(
            agent_spec.task), starting_position=agent_spec.starting_position, starting_orientation=agent_spec.starting_orientation)
        return SimpleNavScenario(world=load_world(spec.world, agents=[agent]), agent=agent)
