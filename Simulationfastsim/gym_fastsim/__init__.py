import logging
from gym.envs.registration import register
from os.path import dirname, join



logger = logging.getLogger(__name__)


default_env = "assets/example.xml"
register(
    id='kitchen-v0',
    entry_point='gym_fastsim.simple_nav:SimpleNavEnv',
    kwargs={"xml_env":join(dirname(__file__), default_env)}
)
