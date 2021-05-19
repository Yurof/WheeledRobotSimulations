from .task import Task
from iRobot_gym.tasks import Task
import numpy as np


class MaximizeProgressTask(Task):

    def __init__(self, time_limit: float, goal_size_detection: float,
                 delta_progress: float = 0.0, collision_reward: float = -10.0,
                 frame_reward: float = 0.0, progress_reward: float = 100.0, n_min_rays_termination=100):
        self._time_limit = time_limit
        self._goal_size_detection = goal_size_detection
        self._n_min_rays_termination = n_min_rays_termination
        self._last_stored_progress = None
        # reward params
        self._delta_progress = delta_progress
        self._progress_reward = progress_reward
        self._collision_reward = collision_reward
        self._frame_reward = frame_reward

    def reward(self, agent_id, state, action) -> float:
        agent_state = state[agent_id]
        progress = agent_state['progress']
        if self._last_stored_progress is None:
            self._last_stored_progress = progress
        delta = (-1)*(progress - self._last_stored_progress)
        reward = self._frame_reward
        reward += delta * self._progress_reward
        self._last_stored_progress = progress
        return reward

    def done(self, agent_id, state) -> bool:
        agent_state = state[agent_id]
        if agent_state['progress'] < self._goal_size_detection:
            return True
        return (self._time_limit < agent_state['time'] and self._time_limit > 0)

    def reset(self):
        self._last_stored_progress = None
