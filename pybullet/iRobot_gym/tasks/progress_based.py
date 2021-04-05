from .task import Task
from iRobot_gym.tasks import Task
import numpy as np


class MaximizeProgressTask(Task):

    def __init__(self, time_limit: float, terminate_on_collision: bool, goal_size_detection: float,
                 delta_progress: float = 0.0, collision_reward: float = -10.0,
                 frame_reward: float = 0.0, progress_reward: float = 100.0, n_min_rays_termination=100):
        self._time_limit = time_limit
        self._terminate_on_collision = terminate_on_collision
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
        if self._check_collision(agent_state):
            reward += self._collision_reward
        reward += delta * self._progress_reward
        self._last_stored_progress = progress
        return reward

    def done(self, agent_id, state) -> bool:
        agent_state = state[agent_id]
        if agent_state['progress'] < self._goal_size_detection:
            return True
        elif self._terminate_on_collision and self._check_collision(agent_state):
            return True
        return self._time_limit < agent_state['time']

    def _check_collision(self, agent_state):
        safe_margin = 0.25
        collision = agent_state['wall_collision'] > 0
        if 'observations' in agent_state and 'lidar' in agent_state['observations']:
            n_min_rays = sum(
                np.where(agent_state['observations']['lidar'] <= safe_margin, 1, 0))
            return n_min_rays > self._n_min_rays_termination or collision
        return collision

    def reset(self):
        self._last_stored_progress = None
