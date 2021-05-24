""" calculate the position and velocity of the robot,
Also the camera that follow the agent (enable in config file)"""

import numpy as np
import pybullet


def get_velocity(id):
    """Ask PyBullet the velocity of the agent.

    Args:
        id: id of the agent.
    Return:
        [x, y, z, roll, pitch, yaw]
    """
    linear, angular = pybullet.getBaseVelocity(id)
    _, orientation = pybullet.getBasePositionAndOrientation(id)
    rotation = pybullet.getMatrixFromQuaternion(orientation)
    rotation = np.reshape(rotation, (-1, 3)).transpose()
    linear = rotation.dot(linear)
    angular = rotation.dot(angular)
    return np.append(linear, angular)


def get_pose(id):
    """Ask PyBullet the position of the agent.

    Args:
        id: id of the agent.
    Return:
        [x, y, z, roll, pitch, yaw]
    """
    position, orientation = pybullet.getBasePositionAndOrientation(id)
    if any(np.isnan(position)) or any(np.isnan(orientation)):
        return None
    orientation = pybullet.getEulerFromQuaternion(orientation)
    pose = np.append(position, orientation)
    return pose


def follow_agent(agent, width=640, height=480):
    """The camera that follow the agent.
    """
    position, orientation = pybullet.getBasePositionAndOrientation(
        agent.vehicle_id)
    _, _, yaw = pybullet.getEulerFromQuaternion(orientation)
    orientation = pybullet.getQuaternionFromEuler((0, 0, yaw))
    rot_matrix = pybullet.getMatrixFromQuaternion(orientation)
    rot_matrix = np.array(rot_matrix).reshape(3, 3)

    camera_position = position + rot_matrix.dot([-0.8, 0, 0.3])
    up_vector = rot_matrix.dot([0, 0, 1])
    target = position

    view_matrix = pybullet.computeViewMatrix(
        camera_position, target, up_vector)

    proj_matrix = pybullet.computeProjectionMatrixFOV(
        fov=60,
        aspect=float(width) / height,
        nearVal=0.1,
        farVal=5.0
    )
    _, _, rgb_image, _, _ = pybullet.getCameraImage(
        width=width,
        height=height,
        renderer=pybullet.ER_BULLET_HARDWARE_OPENGL,
        viewMatrix=view_matrix,
        projectionMatrix=proj_matrix)

    rgb_array = np.reshape(rgb_image, (height, width, -1))
    rgb_array = rgb_array[:, :, :3]
    return rgb_array
