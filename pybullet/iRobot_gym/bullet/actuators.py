from abc import ABC
from dataclasses import dataclass
from typing import Tuple, TypeVar, List

import gym
import numpy as np
import pybullet

from iRobot_gym.core import actuators

T = TypeVar('T')


class BulletActuator(actuators.Actuator[T], ABC):
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
        max_velocity: float
        max_force: float

    def __init__(self, name: str, config: Config):
        super().__init__(name)
        self._config = config

    def control(self, acceleration) -> None:
        acceleration1, acceleration2 =acceleration
        acceleration1 = np.clip(acceleration1, -1, +1)
        if acceleration1 < 0:
            velocity1 = -1*self._config.max_velocity * self._config.velocity_multiplier
        else:
            velocity1 = self._config.max_velocity * self._config.velocity_multiplier
        force1 = abs(acceleration1) * self._config.max_force
        
        acceleration2 = np.clip(acceleration2, -1, +1)
        if acceleration2 < 0:
            velocity2 = -1*self._config.max_velocity * self._config.velocity_multiplier
        else:
            velocity2 = self._config.max_velocity * self._config.velocity_multiplier
        force2 = abs(acceleration2) * self._config.max_force
        #print("velo force",velocity, force)

        #for index in self.joint_indices:
            #print(self.body_id,index)
        pybullet.setJointMotorControl2(
            self.body_id, self.joint_indices[0],
            pybullet.VELOCITY_CONTROL,
            targetVelocity=velocity1,
            force=force1
        )
        pybullet.setJointMotorControl2(
            self.body_id, self.joint_indices[1],
            pybullet.VELOCITY_CONTROL,
            targetVelocity=velocity1,
            force=force1
        )

    def space(self) -> gym.Space:
        return gym.spaces.Box(low=-1.0, high=1.0, shape=(1,), dtype=np.float64)
