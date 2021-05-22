from typing import Dict
import math
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
        if self._scenario.agent.task_name == 'reward_rapprochement_goal':
            self._RewardFunction = RewardRapprochementGoal(
                self._scenario.agent.task_param)
        elif self._scenario.agent.task_name == 'reward_binary_goal_based':
            self._RewardFunction = RewardBinaryGoalBased(
                self._scenario.agent.task_param)
        elif self._scenario.agent.task_name == 'no_reward':
            self._RewardFunction = NoReward(
                self._scenario.agent.task_param)
        elif self._scenario.agent.task_name == 'reward_displacement':
            self._RewardFunction = RewardDisplacement(
                self._scenario.agent.task_param)
        else:
            print("Error task not found")

    @ property
    def scenario(self):
        return self._scenario

    def step(self, action: Dict):
        assert self._initialized, 'Reset before calling step'
        state = self._scenario.world.state()
        self.observation, info = self._scenario.agent.step(action=action)
        # self.observation['time'] = self._time
        done = self._RewardFunction.done(self._scenario.agent.id, state)
        reward = self._RewardFunction.reward(
            self._scenario.agent.id, state, action)
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
        self._RewardFunction.reset()
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


class NoReward:
    """ No reward"""

    def __init__(self, param: dict):
        self._time_limit = param['time_limit']
        self._goal_size_detection = param['goal_size_detection']

    def reward(self, agent_id, state, action) -> float:
        return 0

    def done(self, agent_id, state) -> bool:
        agent_state = state[agent_id]
        if agent_state['dist_obj'] < self._goal_size_detection:
            return True
        return self._time_limit < agent_state['time'] and self._time_limit > 0

    def reset(self):
        pass


class RewardBinaryGoalBased:
    """ Reward of 1 is given when close enough to the goal. """

    def __init__(self, param: dict):
        self._time_limit = param['time_limit']
        self._goal_size_detection = param['goal_size_detection']

    def reward(self, agent_id, state, action) -> float:
        agent_state = state[agent_id]
        if agent_state['dist_obj'] < self._goal_size_detection:
            return 1
        return 0

    def done(self, agent_id, state) -> bool:
        agent_state = state[agent_id]
        if agent_state['dist_obj'] < self._goal_size_detection:
            return True
        return self._time_limit < agent_state['time'] and self._time_limit > 0

    def reset(self):
        pass


class RewardDisplacement:
    """ Reward = distance to previous position"""

    def __init__(self, param: dict):
        self._time_limit = param['time_limit']
        self._goal_size_detection = param['goal_size_detection']
        self._last_stored_pos = None

    def reward(self, agent_id, state, action) -> float:
        agent_state = state[agent_id]
        pose = agent_state['pose']
        if self._last_stored_pos is None:
            self._last_stored_pos = pose
        reward = math.sqrt((pose[0] - self._last_stored_pos[0])**2 +
                           (pose[1] - self._last_stored_pos[1])**2)
        self._last_stored_pos = pose
        return reward

    def done(self, agent_id, state) -> bool:
        agent_state = state[agent_id]
        if agent_state['dist_obj'] < self._goal_size_detection:
            return True
        return self._time_limit < agent_state['time'] and self._time_limit > 0

    def reset(self):
        self._last_stored_pos = None


class RewardRapprochementGoal:
    """ Reward when you reduce the distance to the goal"""

    def __init__(self, param: dict):
        self._time_limit = param['time_limit']
        self._goal_size_detection = param['goal_size_detection']
        self._last_stored_progress = None

    def reward(self, agent_id, state, action) -> float:
        agent_state = state[agent_id]
        progress = agent_state['dist_obj']
        if self._last_stored_progress is None:
            self._last_stored_progress = progress
        reward = (-1)*(progress - self._last_stored_progress)
        self._last_stored_progress = progress
        return reward

    def done(self, agent_id, state) -> bool:
        agent_state = state[agent_id]
        if agent_state['dist_obj'] < self._goal_size_detection:
            return True
        return (self._time_limit < agent_state['time'] and self._time_limit > 0)

    def reset(self):
        self._last_stored_progress = None
