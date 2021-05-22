from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Tuple, TypeVar, List, Generic

import gym
import numpy as np
import pybullet


T = TypeVar('T')


class Actuator(ABC, Generic[T]):

    def __init__(self, name: str):
        self._name = name

    @property
    def name(self):
        return self._name


class BulletActuator(Actuator[T], ABC):
    def __init__(self, name: str):
        super().__init__(name)
        self._body_id = None
        self._joint_indices = []

    def reset(self, body_id: int, joint_indices: List[int] = None):
        self._body_id = body_id
        self._joint_indices = joint_indices

    @property
    def body_id(self) -> int:
        return self._body_id

    @property
    def joint_indices(self) -> List[int]:
        return self._joint_indices


class Motor(BulletActuator[Tuple[float, float]]):
    @dataclass
    class Config:
        velocity_multiplier: float
        max_force: float

    def __init__(self, name: str, config: Config):
        super().__init__(name)
        self._config = config

    def control(self, acceleration) -> None:
        acceleration1, acceleration2 = acceleration

        acceleration1 = np.clip(acceleration1, -1, +1)  # left wheel
        velocity1 = acceleration1 * self._config.velocity_multiplier

        acceleration2 = np.clip(acceleration2, -1, +1)  # right wheel
        velocity2 = acceleration2 * self._config.velocity_multiplier

        pybullet.setJointMotorControlArray(
            self.body_id,
            self.joint_indices,
            pybullet.VELOCITY_CONTROL,
            targetVelocities=[velocity2, velocity1],
            forces=[self._config.max_force]*2
        )

    def space(self) -> gym.Space:
        return gym.spaces.Box(low=-1.0, high=1.0, shape=(2,), dtype=np.float64)
