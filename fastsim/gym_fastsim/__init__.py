import logging
from gym.envs.registration import register
from os.path import dirname, join


logger = logging.getLogger(__name__)


kitchen_env = "assets/kitchen.xml"
kitchen_lasers_env = "assets/kitchen_lasers.xml"
maze_env = "assets/LS_maze_hard.xml"
race_track_env = "assets/race_track.xml"

try:
    register(id='kitchen-v0', entry_point='gym_fastsim.simple_nav:SimpleNavEnv',
             kwargs={"xml_env": join(dirname(__file__), kitchen_env)})
    register(id='kitchen-v1', entry_point='gym_fastsim.simple_nav:SimpleNavEnv',
             kwargs={"xml_env": join(dirname(__file__), kitchen_lasers_env)})
    register(id='maze-v0', entry_point='gym_fastsim.simple_nav:SimpleNavEnv',
             kwargs={"xml_env": join(dirname(__file__), maze_env)})
    register(id='race_track-v0', entry_point='gym_fastsim.simple_nav:SimpleNavEnv',
             kwargs={"xml_env": join(dirname(__file__), race_track_env)})
except:
    pass
