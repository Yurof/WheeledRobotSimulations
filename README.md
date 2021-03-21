# Prandroide
Apprentissage et généralisation sur une tâche de navigation d'un robot à roues 

## installation
lancer `install-dependencies.sh` pour installer les librairies nécessaires :
- gym
- libfastsim
- pybind11
- pyfastsim
- pybullet-gym


## Scenes

| Image | 3D Render | Name |
| --- | --- |--- |
|![kitchen](readme_assets/kitchen.svg)|![kitchen3D](readme_assets/kitchen3D.png)|`Kitchen-v0`|
|![maze_hard](readme_assets/maze_hard.svg)|![maze_hard3D](readme_assets/maze_hard3D.png)|`Maze_hard-v0`|

Use the function ``svg2obj.py`` to convert an svg image to a 3D object using Blender (Note: change directory in the program)
## Robot
![iRobot](readme_assets/irobot.png)

We have modified a model of the iRobot create.
# libfastsim
## Documentation
documentation can be found here : https://github.com/jbmouret/libfastsim
and here for the pytohn binding : https://github.com/alexendy/pyfastsim
## Example
```
python RobotAléatoire.py
```

# Pybullet

## Installation
You can install ``racecar_gym`` with the following commands:

```shell_script
cd pybullet
pip install -e .
```

## Environments


### Actions


|Key|Space|Description|
|---|---|---|
|motor|`Box(low=-1, high=1, shape=(2,))`|Throttle command for each weel. If negative, the car accelerates backwards.|


## Example
```
python simple_usage.py
```
``` python
import gym
from time import sleep
from iRobot_gym.envs import SingleAgentRaceEnv

env = gym.make('Kitchen_Gui-v0')

done = False
obs = env.reset()
t = 0

while not done:
    action = env.action_space.sample()
    obs, rewards, done, states = env.step(action)
    sleep(0.01)
    print("Step  %d reward=%f robot position=%s dist_obj=%f" % (t,rewards,  str(states["pose"][0:3]) ,states["progress"] ) , end="\r" )
    image = env.render()
    t+=1

env.close()
```