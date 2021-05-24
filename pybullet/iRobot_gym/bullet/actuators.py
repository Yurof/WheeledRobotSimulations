"""This module is responsible for the movement of the robot."""

from dataclasses import dataclass
import gym
import numpy as np
import pybullet


class BulletActuator:
    def __init__(self, name):
        """Inits BulletActuator with the attributes values."""
        self._name = name
        self._body_id = None
        self._joint_indices = []

    def reset(self, body_id, joint_indices):
        """reset body id and joint indices."""
        self._body_id = body_id
        self._joint_indices = joint_indices

    @property
    def body_id(self):
        "return the body id."
        return self._body_id

    @property
    def joint_indices(self):
        "return the joint indices."
        return self._joint_indices


class Motor(BulletActuator):
    """This class use PyBullet to make the wheels spins.

    Attributes:
        _name: String the name of the Motor.
        _config: config contening the velocity_multiplier and the max_force.
    """
    @dataclass
    class Config:
        velocity_multiplier: float
        max_force: float

    def __init__(self, name, config):
        """Inits Motor with the attributes values."""
        self.name = name
        self._config = config

    def control(self, acceleration):
        """clip the acceleration to not exceed [-1,1] then multiply it and apply the forces on the joints.

        Args:
            acceleration: A tuple contening the speed of the left and right wheel.

        Returns:
            None
        """
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

    def space(self):
        """Descibe the format of valid action for gym."""
        return gym.spaces.Box(low=-1.0, high=1.0, shape=(2,), dtype=np.float64)
