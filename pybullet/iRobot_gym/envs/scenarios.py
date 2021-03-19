from dataclasses import dataclass
from typing import Dict

from iRobot_gym.bullet import load_world, load_vehicle
from iRobot_gym.tasks import Task, get_task
from iRobot_gym.core import World, Agent
from .specs import ScenarioSpec, TaskSpec

def task_from_spec(spec: TaskSpec) -> Task:
    task = get_task(spec.task_name)
    return task(**spec.params)



@dataclass
class SingleAgentScenario:
    world: World
    agent: Agent

    @staticmethod
    def from_spec(path: str, rendering: bool = None) -> 'SingleAgentScenario':
        spec = ScenarioSpec()
        spec.load(path)
        if rendering:
            spec.world.rendering = rendering
        agent_spec = spec.agents[0]
        agent = Agent(id=agent_spec.id, vehicle=load_vehicle(agent_spec.vehicle), task=task_from_spec(agent_spec.task))

        return SingleAgentScenario(world=load_world(spec.world, agents=[agent]), agent=agent)
