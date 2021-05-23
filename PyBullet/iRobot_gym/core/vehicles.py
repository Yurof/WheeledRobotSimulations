from abc import ABC, abstractmethod
from typing import Any, Dict, List

import gym

from .actuators import Actuator
from .definitions import Pose
from .sensors import Sensor
import numpy as np


class Vehicle(ABC):

    def control(self, commands: Dict) -> None:
        self.actuators['motor'].control(commands)

    def observe(self) -> Dict[str, Any]:
        #observations = {}
        L_obs = []
        for sensor in self.sensors:
            #observations[sensor.name] = sensor.observe()
            if sensor.name == 'left_bumper' or sensor.name == 'right_bumper':
                L_obs.append(int(np.round(sensor.observe(), 2) == 0))
            else:
                L_obs = sensor.observe().tolist()
        return L_obs

    @property
    @abstractmethod
    def id(self) -> Any:
        pass

    @property
    @abstractmethod
    def sensors(self) -> List[Sensor]:
        pass

    @property
    @abstractmethod
    def actuators(self) -> Dict[str, Actuator]:
        pass

    @property
    def action_space(self) -> gym.spaces.Dict:
        return gym.spaces.Dict(dict((name, actuator.space()) for name, actuator in self.actuators.items()))

    @property
    def observation_space(self) -> gym.spaces.Dict:
        return gym.spaces.Dict(dict((s.name, s.space()) for s in self.sensors))

    @abstractmethod
    def reset(self, pose: Pose):
        pass
