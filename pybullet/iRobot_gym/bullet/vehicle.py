"""This class define the robot, it links the joints of the urdf file to the wheel and the laser."""

from dataclasses import dataclass
import gym
import pybullet


class IRobot:
    """This class define the robot, it links the joints of the urdf file to the wheel and the laser.
    Attributes:
        _id: id of the agent.
        _config: config contening the urdf file of the agent.
        _actuators: dict representing the different actuators.
        _sensors: object representing the sensors.
    """
    @dataclass
    class Config:
        urdf_file: str

    def __init__(self, sensors, actuators, config):
        """Inits IRobot with the attributes values."""
        self._id = None
        self._config = config
        self._actuators = dict([(a.name, a) for a in actuators])
        self._sensors = sensors

        self._sensor_indices = {
            'laser': 4
        }
        self._actuator_indices = {
            'motor': [0, 1]
        }

    def control(self, commands):
        self.actuators['motor'].control(commands)

    def observe(self):
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

    def _load_model(self, model, initial_pose):
        position, orientation = initial_pose
        orientation = pybullet.getQuaternionFromEuler(orientation)
        return pybullet.loadURDF(model, position, orientation)

    def _get_info(self):
        for k in range(pybullet.getNumJoints(id)):
            print("ID", id, pybullet.getJointInfo(id, k))
            print("dynamic: ", pybullet.getDynamicsInfo(id, k))
        print("dynamic: ", pybullet.getDynamicsInfo(id, -1))

    @property
    def id(self):
        return self._id

    @property
    def sensors(self):
        return self._sensors

    @property
    def actuators(self):
        return self._actuators

    @property
    def action_space(self):
        return gym.spaces.Dict(dict((name, actuator.space()) for name, actuator in self.actuators.items()))

    @property
    def observation_space(self):
        return gym.spaces.Dict(dict((s.name, s.space()) for s in self.sensors))
