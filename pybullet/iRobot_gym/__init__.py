import os
from gym.envs.registration import register
from .envs import SingleAgentScenario, SingleAgentRaceEnv
from .tasks import get_task, register_task, Task

base_path = os.path.dirname(__file__)



def _register_single_agent(name: str, file: str, rendering: bool):
    print("\n\nregister\n\n",f'{base_path}/../scenarios/{file}')
    scenario = SingleAgentScenario.from_spec(path=f'{base_path}/../scenarios/{file}', rendering=rendering)
    register(
        id=name,
        entry_point='iRobot_gym.envs:SingleAgentRaceEnv',
        kwargs={'scenario': scenario}
    )

for scenario_file in os.listdir(f'{base_path}/../scenarios'):
    track_name = os.path.basename(scenario_file).split('.')[0]
    name = f'{track_name.capitalize()}'
    _register_single_agent(name=f'{name}-v0', file=scenario_file, rendering=False)
    _register_single_agent(name=f'{name}_Gui-v0', file=scenario_file, rendering=True)


