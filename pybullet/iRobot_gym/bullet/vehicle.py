"""This class define the robot, it links the joints of the urdf file to the wheel and the laser  """

from dataclasses import dataclass
from typing import List, Dict, Any

import gym
from iRobot_gym.bullet.actuators import BulletActuator
from iRobot_gym.bullet.sensors import BulletSensor
import pybullet


class IRobot:
    @dataclass
    class Config:
        urdf_file: str

    def __init__(self, sensors: List[BulletSensor], actuators: List[BulletActuator], config: Config):
        super().__init__()
        self._id = None
        self._config = config

        self._sensor_indices = {
            'lidar': 4
        }

        self._actuator_indices = {
            'motor': [0, 1]
        }
        self._actuators = dict([(a.name, a) for a in actuators])
        self._sensors = sensors

    def control(self, commands: Dict) -> None:
        self.actuators['motor'].control(commands)

    def observe(self) -> Dict[str, Any]:
        observations = []
        for sensor in self.sensors:
            observations = sensor.observe()
        return observations

    def reset(self, pose):
        if not self._id:
            self._id = self._load_model(
                self._config.urdf_file, initial_pose=pose)
        else:
            pos, orn = pose
            pybullet.resetBasePositionAndOrientation(
                self._id, pos, pybullet.getQuaternionFromEuler(orn))

        for sensor in self.sensors:
            joint_index = None
            if sensor.type in self._sensor_indices:
                joint_index = self._sensor_indices[sensor.type]
            sensor.reset(body_id=self._id, joint_index=joint_index)

        for name, actuator in self.actuators.items():
            joint_indices = None
            if name in self._actuator_indices:
                joint_indices = self._actuator_indices[name]
            actuator.reset(body_id=self._id, joint_indices=joint_indices)

    def _load_model(self, model: str, initial_pose) -> int:
        position, orientation = initial_pose
        orientation = pybullet.getQuaternionFromEuler(orientation)
        # pybullet.changeDynamics(id, 0, spinningFriction=0.1)
        # pybullet.changeDynamics(id, 1, spinningFriction=0.1)

        # for k in range(pybullet.getNumJoints(id)):
        #     print("\nID", id, pybullet.getJointInfo(id, k))
        #     print("dynamic: ", pybullet.getDynamicsInfo(id, k))

        # print("\ndynamic: ", pybullet.getDynamicsInfo(id, -1))
        return pybullet.loadURDF(model, position, orientation)

    @property
    def id(self) -> Any:
        return self._id

    @property
    def sensors(self) -> List[BulletSensor]:
        return self._sensors

    @property
    def actuators(self) -> Dict[str, BulletActuator]:
        return self._actuators

    @property
    def action_space(self) -> gym.spaces.Dict:
        return gym.spaces.Dict(dict((name, actuator.space()) for name, actuator in self.actuators.items()))

    @property
    def observation_space(self) -> gym.spaces.Dict:
        return gym.spaces.Dict(dict((s.name, s.space()) for s in self.sensors))
