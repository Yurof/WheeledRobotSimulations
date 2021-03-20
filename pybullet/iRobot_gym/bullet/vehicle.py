from dataclasses import dataclass
from typing import List, Dict, Any, Tuple

import pybullet

from iRobot_gym.bullet.actuators import BulletActuator
from iRobot_gym.bullet.sensors import BulletSensor
from iRobot_gym.core.definitions import Pose
from iRobot_gym.core.vehicles import Vehicle


class IRobot(Vehicle):
    @dataclass
    class Config:
        urdf_file: str

    def __init__(self, sensors: List[BulletSensor], actuators: List[BulletActuator], config: Config):
        super().__init__()
        self._id = None
        self._config = config
        self._on_finish = False

        self._sensor_indices = {
            'lidar': 4
        }

        self._actuator_indices = {
            'motor': [0, 1]
        }
        self._actuators = dict([(a.name, a) for a in actuators])
        self._sensors = sensors

    @property
    def id(self) -> Any:
        return self._id

    @property
    def sensors(self) -> List[BulletSensor]:
        return self._sensors

    @property
    def actuators(self) -> Dict[str, BulletActuator]:
        return self._actuators

    def reset(self, pose: Pose):
        if not self._id:
            self._id = self._load_model(self._config.urdf_file, initial_pose=pose)
        else:
            pos, orn = pose
            pybullet.resetBasePositionAndOrientation(self._id, pos, pybullet.getQuaternionFromEuler(orn))

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

    def _load_model(self, model: str, initial_pose: Pose) -> int:
        position, orientation = initial_pose
        orientation = pybullet.getQuaternionFromEuler(orientation)
        id = pybullet.loadURDF(model, position, orientation)
        #pybullet.changeVisualShape(id, -1, rgbaColor=self._config.color)
        
        # for k in range(pybullet.getNumJoints(id)):
        #     print("ID",id, pybullet.getJointInfo(id,k))
        
        return id
