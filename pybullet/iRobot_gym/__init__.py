import os
from gym.envs.registration import register
from .envs import SimpleNavScenario, SimpleNavEnv

base_path = os.path.dirname(__file__)


def _register_single_agent(name: str, file: str):
    scenario = SimpleNavScenario.from_spec(
        path=f'{base_path}/../configuration/scenarios/{file}')
    register(
        id=name,
        entry_point='iRobot_gym.envs:SimpleNavEnv',
        kwargs={'scenario': scenario}
    )


for scenario_file in os.listdir(f'{base_path}/../configuration/scenarios'):
    track_name = os.path.basename(scenario_file).split('.')[0]
    name = f'{track_name}'
    _register_single_agent(
        name=f'{name}-v0', file=scenario_file)
