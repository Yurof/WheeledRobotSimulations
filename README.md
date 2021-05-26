# Prandroide

Learning and generalization on a navigation task of a wheeled robot.

The objective of this project is to develop several simulation environments  for wheeled robots based on existing simulation libraries. The aim is to highlight the problems of transferring learned policies between different simulators. The first simulation environment will be based on the fast simulation library libfastsim. The second environment will be more realistic and will be based on the pybullet library.

## demonstration
<div style="text-align:center"><img src="assets/readme/pybullet2.gif" width="400" height="400"/></div>

## installation

Run `install-dependencies.sh` to install the necessary libraries:

- gym
- libfastsim
- pybind11
- pyfastsim
- pybullet-gym
- deap

boost and SDL 1.2 may be necessary for libfastsim.

## Scenes

| Image                                      | 3D Render                                      | Name            |
| ------------------------------------------ | ---------------------------------------------- | --------------- |
| ![kitchen](assets/readme/kitchen.svg)      | ![kitchen3D](assets/readme/kitchen3D.png)      | `kitchen-v0`    |
| ![maze_hard](assets/readme/maze_hard.svg)  | ![maze_hard3D](assets/readme/maze_hard3D.png)  | `maze_hard-v0`  |
| ![maze_hard](assets/readme/race_track.svg) | ![maze_hard3D](assets/readme/race_track3D.png) | `race_track-v0` |

```console
blender --background --python pbm_to_obj.py
```

in the blender folder to convert an pbm image to a 3D object using Blender and Potrace.

## Robot

<div style="text-align:center"><img src="assets/readme/irobot_create.png" width="300"></div>

We have modified a model of the iRobot create.

# libfastsim

## Documentation

documentation can be found here : https://github.com/jbmouret/libfastsim
and here for the python binding : https://github.com/alexendy/pyfastsim

# Pybullet

## Installation

You can install `irobot_gym` with the following commands:

```shell_script
cd pybullet
pip install -e .
```

### Configuration

each scene is configured in a yml file with its name in the "scenarios" folder like so:

```yml
world:
  name: race_track
  sdf: race_track.urdf
  scale: 20 #size of the map in meter
  physics:
    gravity: -9.81
  simulation:
    time_step: 0.016666 #240Hz
    GUI: True
    following_camera: False
  goal:
    goal_position: [2, 5, 0] #x,y,z
    goal_size: 0.2 #diameter of the cylinder

agents:
  id: A
  vehicle:
    name: iRobot
    sensors: [laser]
  task:
    task_name: maximize_progress #type of reward function
    params: { time_limit: 1000.0, goal_size_detection: 0.2 }
  starting_position: [3, 4, 0] #x,y,z
  starting_orientation: [0, 0, 0] #roll,pitch yaw
```

And for the configuration of the robot:

```yml
urdf_file: iRobot.urdf

actuators:
  - type: motor
    name: motor
    params:
      velocity_multiplier: 16.5 # 0.5m/s
      max_force: 20

sensors:
  - type: lidar
    name: lidar
    frequency: 100
    params:
      inaccuracy: 0.00
      rays: 10 # number of lasers
      range: 1 # range in meter
      min_range: 0 # laser start offset
      angle_start: -90 # degree
      angle: 180
      debug: True # visible laser of not
```

# Run

In the `pybullet` and `fastsim` folder we have a `main.py` file used to run each simulation.
Witch take for arguments:

- `--env` : environnement: `kitchen`, `maze_hard`, `race_track`.
- `--ctr` : choose controller: `forward`, `wall`, `rule`, `braitenberg`, `novelty`.
- `--sleep_time` : sleeping time between each step.
- `--save_res` : save the result in a csv file: True or False.
- `--verbose` : verbose for the controller: True or False.
- `--file_name` : file name of the invidual to load if `--ctr`=`novelty`.

# Gym

`step` : [v1,v2] to move the left and right whell.

`observation` : a list giving the value of each laser.

`info` :
| info['s'] | description | return |
|--------------|----------------------------|-----------------------------|
| pose | position of the robot | [x, y, z, roll, pitch, yaw] |
| acceleration | acceleration of the robot | [x, y, z, roll, pitch, yaw] |
| velocity | velocity of the robot | [x, y, z, roll, pitch, yaw] |
| dist_obj | distance to the objectif | a float |
| time | simulation time | a float |

`reward`, we have different reward functions that can be changed:

- `reward_binary_goal_based`: a reward of 1 is given when you get close enough to the goal.
- `reward_displacement` : the reward corresponds to the distance from the previous position.
- `reward_rapprochement_goal` : the reward corresponds to the distance to the goal.
- `no_reward`: no reward.
