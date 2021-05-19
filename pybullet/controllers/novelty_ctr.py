from controllers.fixed_structure_nn_numpy import SimpleNeuralControllerNumpy
import pickle
import os
from deap import *
import array
from deap import algorithms
from deap import base
from deap import benchmarks
from deap import creator
from deap import tools


class NoveltyController:

    def __init__(self, env, file):

        self.env = env
        self.nn = SimpleNeuralControllerNumpy(*[10, 2, 2, 10])
        base_path = os.path.dirname(os.path.abspath(__file__))

        creator.create("MyFitness", base.Fitness, weights=(-1.0,))
        creator.create("Individual", array.array, typecode="d",
                       fitness=creator.MyFitness, strategy=None)
        creator.create("Strategy", array.array, typecode="d")

        f = open(f"{base_path}/../../results/individuals/{file}.pkl", "rb")
        indmin = pickle.load(f)

        self.nn.set_parameters(indmin)

    def get_command(self):

        sensors = self.env.get_laserranges()
        action = self.nn.predict(sensors)
        return action

    def reset(self):
        pass
