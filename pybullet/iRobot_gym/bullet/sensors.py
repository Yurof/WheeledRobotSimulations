from abc import ABC
from dataclasses import dataclass
from typing import Any, TypeVar, Tuple, Union

import gym
import numpy as np
import pybullet as p
from nptyping import NDArray

from iRobot_gym.bullet import util
from iRobot_gym.core import Sensor

T = TypeVar('T')


class BulletSensor(Sensor[T], ABC):

    def __init__(self, name: str, type: str):
        super().__init__(name, type)
        self._body_id = None
        self._joint_index = None

    def reset(self, body_id: int, joint_index: int = None):
        self._body_id = body_id
        self._joint_index = joint_index

    @property
    def body_id(self) -> int:
        return self._body_id

    @property
    def joint_index(self) -> int:
        return self._joint_index


class FixedTimestepSensor(BulletSensor[T], ABC):

    def __init__(self, sensor: BulletSensor, frequency: float, time_step: float):
        super().__init__(sensor.name, sensor.type)
        self._sensor = sensor
        self._frequency = 1.0 / frequency
        self._time_step = time_step
        self._last_timestep = 0
        self._last_observation = None

    def space(self) -> gym.Space:
        return self._sensor.space()

    def observe(self) -> T:
        self._last_timestep += self._time_step
        if self._last_timestep >= self._frequency or self._last_observation is None:
            self._last_observation = self._sensor.observe()
            self._last_timestep = 0
        return self._last_observation

    def reset(self, body_id: int, joint_index: int = None):
        self._sensor.reset(body_id=body_id, joint_index=joint_index)


class Lidar(BulletSensor[NDArray[(Any,), np.float]]):
    @dataclass
    class Config:
        accuracy: float
        rays: int
        range: float
        angle_start: float
        angle: float
        min_range: float
        debug: bool = True

    def __init__(self, name: str, type: str, config: Config):
        super().__init__(name, type)
        self._config = config
        self._min_range = config.min_range
        self._rays = self._config.rays
        self._range = self._config.range
        self._hit_color = [1, 0, 0]
        self._miss_color = [0, 1, 0]
        self._ray_ids = []

        self._from, self._to = self._setup_raycast(min_distance=self._min_range,
                                                   scan_range=self._range,
                                                   rays=self._rays)

    def _setup_raycast(self, min_distance: float, scan_range: float, rays: int):
        start = min_distance
        end = min_distance + scan_range
        from_points, to_points = [], []
        angle = self._config.angle_start + np.pi / 2.0
        increment = self._config.angle / self._config.rays
        for i in range(rays):
            from_points.append([
                start * np.sin(angle),
                start * np.cos(angle),
                0
            ])

            to_points.append([
                end * np.sin(angle),
                end * np.cos(angle),
                0
            ])
            angle += increment

        return np.array(from_points), np.array(to_points)

    def space(self) -> gym.Space:
        return gym.spaces.Box(low=self._min_range,
                              high=self._min_range + self._range,
                              dtype=np.float64,
                              shape=(self._rays,))

    def observe(self) -> NDArray[(Any,), np.float]:
        results = p.rayTestBatch(self._from, self._to, 0,
            parentObjectUniqueId=self.body_id,
            parentLinkIndex=self.joint_index)
        hit_fractions = np.array(results, dtype=np.object)[:, 2].astype(dtype=np.float)
        ranges = self._config.range * hit_fractions + self._config.min_range
        noise = np.random.uniform(1.0 - self._config.accuracy, 1.0 + self._config.accuracy, size=ranges.shape)
        scan = np.clip(ranges * noise, a_min=self._config.min_range, a_max=self._config.range)

        if self._config.debug:
            self._display_rays(hit_fractions, scan)

        return scan

    def _display_rays(self, hit_fractions, scan):
        angle = self._config.angle_start + np.pi / 2.0
        increment = self._config.angle / self._config.rays
        for i in range(self._rays):
            if len(self._ray_ids) < self._rays:
                ray_id = p.addUserDebugLine(self._from[i], self._to[i], self._miss_color,
                    parentObjectUniqueId=self.body_id,
                    parentLinkIndex=self.joint_index)
                self._ray_ids.append(ray_id)

            if (hit_fractions[i] == 1.):
                color = self._miss_color
            else:
                color = self._hit_color

            localHitTo = [
                self._from[i][0] + scan[i] * np.sin(angle),
                self._from[i][1] + scan[i] * np.cos(angle),
                self._from[i][2]
            ]

            p.addUserDebugLine(
                self._from[i],
                localHitTo,
                color,
                replaceItemUniqueId=self._ray_ids[i],
                parentObjectUniqueId=self.body_id,
                parentLinkIndex=self.joint_index
            )

            angle += increment

