from typing import Dict
import gym
from .scenarios import SimpleNavScenario


class SimpleNavEnv(gym.Env):

    def __init__(self, scenario: SimpleNavScenario):
        self._scenario = scenario
        self._initialized = False
        self._time = 0.0
        self.observation_space = scenario.agent.observation_space
        self.action_space = scenario.agent.action_space
        self.observation = dict()

    @property
    def scenario(self):
        return self._scenario

    def step(self, action: Dict):
        assert self._initialized, 'Reset before calling step'
        state = self._scenario.world.state()
        self.observation, info = self._scenario.agent.step(action=action)
        # self.observation['time'] = self._time
        done = self._scenario.agent.done(state)
        reward = self._scenario.agent.reward(state, action)
        self._time = self._scenario.world.update(
            agent_id=self._scenario.agent.id)
        return self.observation, reward, done, state[self._scenario.agent.id]

    def reset(self):
        if not self._initialized:
            self._scenario.world.init()
            self._initialized = True
        else:
            self._scenario.world.reset()
        obs = self._scenario.agent.reset(
            self._scenario.world.get_starting_position(self._scenario.agent))
        self._scenario.world.update(agent_id=self._scenario.agent.id)
        return obs

    def render(self, **kwargs):
        return self._scenario.world.render(agent_id=self._scenario.agent.id, **kwargs)

    def seed(self, seed=None):
        self._scenario.world.seed(seed)

    def get_laserranges(self):
        return self.observation

    def logfile(self):
        return
        # self._scenario.world.loginfo()
