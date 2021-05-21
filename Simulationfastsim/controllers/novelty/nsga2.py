from deap import *
import numpy as np
from fixed_structure_nn_numpy import SimpleNeuralControllerNumpy
import cma
import gym
import gym_fastsim


from deap import algorithms
from deap import base
from deap import benchmarks
from deap import creator
from deap import tools

import argparse
import array
import random
import operator
import math
import pickle
import os
import time
from scoop import futures

from novelty_search import *


def eval_nn(genotype, env, nbstep=5000, render=False, name="", nn_size=[10, 2, 2, 10]):
    nn = SimpleNeuralControllerNumpy(*nn_size)
    nn.set_parameters(genotype)
    observation = env.reset()
    old_pos = None
    total_dist = 0

    for t in range(nbstep):
        if render:
            env.render()

        action = nn.predict(observation)
        observation, reward, done, info = env.step(action/117)
        pos = info["robot_pos"][:2]

        if (old_pos is not None):
            d = math.sqrt((pos[0]-old_pos[0])**2+(pos[1]-old_pos[1])**2)
            total_dist += d
        old_pos = list(pos)
        if(done):
            print("X", end="", flush=True)
            break

    dist_obj = info["dist_obj"]
    rpos = [round(x, 2) for x in pos]

    return round(dist_obj, 2), rpos


# Individual generator
def generateES(icls, scls, size, imin, imax, smin, smax):
    ind = icls(random.uniform(imin, imax) for _ in range(size))
    ind.strategy = scls(random.uniform(smin, smax) for _ in range(size))
    return ind


def checkStrategy(minstrategy):
    def decorator(func):
        def wrappper(*args, **kargs):
            children = func(*args, **kargs)
            for child in children:
                for i, s in enumerate(child.strategy):
                    if s < minstrategy:
                        child.strategy[i] = minstrategy
            return children
        return wrappper
    return decorator


creator.create("MyFitness", base.Fitness, weights=(-1.0,))
creator.create("Individual", array.array, typecode="d",
               fitness=creator.MyFitness, strategy=None)
creator.create("Strategy", array.array, typecode="d")


def launch_nsga2(environment, mu=100, lambda_=100, ngen=2, nn_size=[10, 2, 2, 10], variant="NS"):
    random.seed()

    nn = SimpleNeuralControllerNumpy(*nn_size)
    params = nn.get_parameters()
    IND_SIZE = len(params)
    MIN_VALUE = -30
    MAX_VALUE = 30
    MIN_STRATEGY = 0.5
    MAX_STRATEGY = 3
    cxpb = 0.6
    mutpb = 0.3

    if variant == "FIT":
        weights = (-1.0,)
    elif variant == "NS":
        weights = (1.0,)
    else:
        weights = (-1.0, 1.0)

    creator.create("MyFitness", base.Fitness, weights=weights)
    creator.create("Individual", array.array, typecode="d",
                   fitness=creator.MyFitness, strategy=None)
    creator.create("Strategy", array.array, typecode="d")

    # définition de la toolbox
    toolbox = base.Toolbox()
    toolbox.register("individual", generateES, creator.Individual, creator.Strategy,
                     IND_SIZE, MIN_VALUE, MAX_VALUE, MIN_STRATEGY, MAX_STRATEGY)
    toolbox.register("population", tools.initRepeat, list, toolbox.individual)
    toolbox.register("map", futures.map)
    toolbox.register("mate", tools.cxSimulatedBinaryBounded,
                     low=MIN_VALUE, up=MAX_VALUE, eta=20.0)
    toolbox.register("mutate", tools.mutPolynomialBounded,
                     low=MIN_VALUE, up=MAX_VALUE, eta=20.0, indpb=1.0 / IND_SIZE)
    toolbox.decorate("mate", checkStrategy(MIN_STRATEGY))
    toolbox.decorate("mutate", checkStrategy(MIN_STRATEGY))
    toolbox.register("evaluate", eval_nn, env=environment)
    toolbox.register("select", tools.selNSGA2)

    # création de la population
    population = toolbox.population(n=mu)

    paretofront = tools.ParetoFront()

    # pour sauvegarder la position finale des politiques explorées
    fbd = open(f"bd-{file_name}.log", "w")

    # Evaluate the individuals with an invalid fitness
    invalid_ind = [ind for ind in population if not ind.fitness.valid]
    fitnesses_bds = toolbox.map(toolbox.evaluate, invalid_ind)

    for ind, (fit, bd) in zip(invalid_ind, fitnesses_bds):
        if (variant == "FIT+NS"):
            ind.fitness.values = (fit, 0)
        elif (variant == "FIT"):
            ind.fitness.values = (fit,)
        elif (variant == "NS"):
            ind.fitness.values = (0,)
        ind.fit = fit
        ind.bd = bd
        fbd.write(" ".join(map(str, bd))+"\n")
        fbd.flush()

    population = toolbox.select(population, len(population))
    if paretofront is not None:
        paretofront.update(population)
    #print("Pareto Front: "+str(paretofront))

    k = 15
    add_strategy = "random"
    lambdaNov = 6

    if variant == 'NS' or variant == 'FIT+NS':
        archive = updateNovelty(population, population,
                                None, k, add_strategy, lambdaNov)

    for ind in population:
        if (variant == "FIT+NS"):
            ind.fitness.values = (ind.fit, ind.novelty)
        elif (variant == "FIT"):
            ind.fitness.values = (ind.fit,)
        elif (variant == "NS"):
            ind.fitness.values = (ind.novelty,)

    indexmin, valuemin = min(
        enumerate([i.fit for i in population]), key=operator.itemgetter(1))

    # Begin the generational process
    for gen in range(1, ngen + 1):
        if (gen % 10 == 0):
            print(gen, end="", flush=True)
        else:
            print(".", end="", flush=True)

        # générer un ensemble de points à partir de la population courante:
        offspring = algorithms.varOr(
            population, toolbox, lambda_, cxpb=cxpb, mutpb=mutpb)

        # Evaluate the individuals with an invalid fitness
        invalid_ind = [ind for ind in offspring if not ind.fitness.valid]
        fitnesses_bds = toolbox.map(toolbox.evaluate, invalid_ind)

        for ind, (fit, bd) in zip(invalid_ind, fitnesses_bds):
            if (variant == "FIT+NS"):
                ind.fitness.values = (fit, 0)
            elif (variant == "FIT"):
                ind.fitness.values = (fit,)
            elif (variant == "NS"):
                ind.fitness.values = (0,)
            ind.fit = fit
            ind.bd = bd
            fbd.write(" ".join(map(str, bd))+"\n")
            fbd.flush()

        pq = population+offspring

        if variant == 'NS' or variant == 'FIT+NS':
            archive = updateNovelty(
                pq, offspring, archive, k, add_strategy, lambdaNov)

        for ind in pq:
            if (variant == "FIT+NS"):
                ind.fitness.values = (ind.fit, ind.novelty)
            elif (variant == "FIT"):
                ind.fitness.values = (ind.fit,)
            elif (variant == "NS"):
                ind.fitness.values = (ind.novelty,)

        # choisir la nouvelle population à partir de pq
        population = toolbox.select(pq, mu)
        # Update the hall of fame with the generated individuals
        if paretofront is not None:
            paretofront.update(population)

        if (gen % 10 == 0):
            for i, p in enumerate(paretofront):
                f = open(
                    f"{base_path}/../../../results/individuals/{file_name}-gen{str(gen)}-p{str(i)}.pkl", "wb")
                pickle.dump(p, f)
                break

        indexmin, newvaluemin = min(
            enumerate([i.fit for i in pq]), key=operator.itemgetter(1))

        if (newvaluemin < valuemin):
            valuemin = newvaluemin
            print("Gen "+str(gen)+", new min ! min fit=" +
                  str(valuemin)+" index="+str(indexmin))
            dist_obj, rpos = eval_nn(pq[indexmin], environment,
                                     render=False, name="gen%04d" % (gen))
            if valuemin < 0.2:
                for i, p in enumerate(paretofront):
                    print("Visualizing indiv "+str(i) +
                          ", fit="+str(p.fitness.values))
                    f = open(
                        f"{base_path}/../../../results/individuals/{file_name}-gen{str(gen)}-p{str(i)}.pkl", "wb")
                    eval_nn(p, env, name=str(i), render=False)
                    pickle.dump(p, f)
                break

        if (gen == ngen):
            for i, p in enumerate(paretofront):
                print("Visualizing indiv "+str(i) +
                      ", fit="+str(p.fitness.values))
                f = open(
                    f"{base_path}/../../../results/individuals/{file_name}-gen{str(gen)}-p{str(i)}.pkl", "wb")
                eval_nn(p, env, name=str(i), render=False)
                pickle.dump(p, f)
    fbd.close()

    # return population, None, paretofront


if (__name__ == "__main__"):

    parser = argparse.ArgumentParser(
        description='Launch maze navigation experiment.')
    parser.add_argument('--env', type=str, default="maze",
                        help='choose between kitchen, maze and race_track')
    parser.add_argument('--nb_gen', type=int, default=200,
                        help='number of generations')
    parser.add_argument('--mu', type=int, default=100,
                        help='population size')
    parser.add_argument('--lambda_', type=int, default=100,
                        help='number of individuals to generate')
    parser.add_argument('--variant', type=str, default="FIT", choices=['FIT', 'NS', 'FIT+NS'],
                        help='variant to consider')
    parser.add_argument('--hidden_layers', type=int, default=2,
                        help='number of hidden layers of the NN controller')
    parser.add_argument('--neurons_per_layer', type=int, default=10,
                        help='number of hidden layers of the NN controller')
    parser.add_argument('--file_name', type=str,
                        default='maze_fit11', help='file name')

    args = parser.parse_args()
    env = args.env+'-v0'
    print("env: ", env)
    env = gym.make(env)
    print("Number of generations: "+str(args.nb_gen))
    ngen = args.nb_gen
    print("Population size: "+str(args.mu))
    mu = args.mu
    print("Number of offspring to generate: "+str(args.lambda_))
    lambda_ = args.lambda_
    print("Variant: "+args.variant)
    variant = args.variant
    print("saved as: ", args.file_name)
    file_name = args.file_name

    print("NN: %d hidden layers with %d neurons per layer" %
          (args.hidden_layers, args.neurons_per_layer))

    start = time.time()
    nn_size = [10, 2, args.hidden_layers, args.neurons_per_layer]
    base_path = os.path.dirname(os.path.abspath(__file__))

    # pop, logbook, paretofront =
    launch_nsga2(env, mu=mu, lambda_=lambda_, ngen=ngen,
                 variant=variant, nn_size=nn_size)

    # for i, p in enumerate(paretofront):
    #     print("Visualizing indiv "+str(i)+", fit="+str(p.fitness.values))
    #     f = open(
    #         f"{base_path}/../../../results/individuals/{file_name}-f{str(i)}.pkl", "wb")
    #     eval_nn(p, env, name=str(i), render=False)
    #     pickle.dump(p, f)
    # f.close()

    env.close()
    print("\n time taken: ", time.time()-start)
