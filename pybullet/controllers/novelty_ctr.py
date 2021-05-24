"""The controller that load an individual class."""

import pickle
import os
import array
from deap import base
from deap import creator
from controllers.fixed_structure_nn_numpy import SimpleNeuralControllerNumpy


class NoveltyController:
    """The controller load a generated individual then use the SimpleNeuralController
    to take the next action.

    Attributes:
        _env: The actual environnment.
        _nn: the neural controller
    """

    def __init__(self, env, file, verbose=False):
        """Inits NoveltyController with the attributes values and load the file"""
        self._env = env
        self._verbose = verbose
        self._nn = SimpleNeuralControllerNumpy(*[10, 2, 2, 10])
        base_path = os.path.dirname(os.path.abspath(__file__))

        creator.create("MyFitness", base.Fitness, weights=(-1.0,))
        creator.create("Individual", array.array, typecode="d",
                       fitness=creator.MyFitness, strategy=None)
        creator.create("Strategy", array.array, typecode="d")

        f = open(f"{base_path}/../../results/individuals/{file}.pkl", "rb")
        self._nn.set_parameters(pickle.load(f))

    def get_command(self):
        """Calculates the futur action of the robot.

        Returns:
            [left, right]: the action for the left and right wheel.
        """
        action = self._nn.predict(self._env.observation)
        if self._verbose:
            print(action)

        return action

    def reset(self):
        pass
