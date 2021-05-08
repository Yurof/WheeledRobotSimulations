from deap import *
import numpy as np
from controllers.novelty.fixed_structure_nn_numpy import SimpleNeuralControllerNumpy

from deap import algorithms
from deap import base
from deap import benchmarks
from deap import creator
from deap import tools

import array
import random
import operator
import math

from scoop import futures

from controllers.novelty.novelty_search import *

        
def eval_nn(genotype, env, nbstep=2000, render=False, name="", nn_size=[12,2,2,10]):
    nn=SimpleNeuralControllerNumpy(*nn_size)
    nn.set_parameters(genotype)
    observation = env.reset()
    old_pos=None
    total_dist=0
    
    if (render):
        f=open("novelty_traj_"+name+".log","w")
        
    for t in range(nbstep):
        if render:
            env.render()
        
        action=nn.predict(observation)
        observation, reward, done, info = env.step(action)
        pos=info["robot_pos"][:2]
        
        if(render and t%10==0):
            f.write(" ".join(map(str,pos))+"\n")
            
        if (old_pos is not None):
            d=math.sqrt((pos[0]-old_pos[0])**2+(pos[1]-old_pos[1])**2)
            total_dist+=d
        old_pos=list(pos)
        if(done):
            break
            
    if (render):
        f.close()
        
    dist_obj=info["dist_obj"]
    rpos=[round(x,2) for x in pos]
    
    return round(dist_obj,2), rpos


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
creator.create("Individual", array.array, typecode="d", fitness=creator.MyFitness, strategy=None)
creator.create("Strategy", array.array, typecode="d")


def launch_nsga2(environment, mu=100, lambda_=100, ngen=200, nn_size=[12,2,2,10], variant="NS"):
    random.seed()

    nn=SimpleNeuralControllerNumpy(*nn_size)
    params=nn.get_parameters()
    IND_SIZE=len(params)
    MIN_VALUE = -30
    MAX_VALUE = 30
    MIN_STRATEGY = 0.5
    MAX_STRATEGY = 3
    cxpb=0.6
    mutpb=0.3
    
    if variant=="FIT":
        weights = (-1.0,)
    elif variant=="NS":
    	weights = (1.0,)
    else:
    	weights = (-1.0, 1.0)
      
    creator.create("MyFitness", base.Fitness, weights=weights)
    creator.create("Individual", array.array, typecode="d", fitness=creator.MyFitness, strategy=None)
    creator.create("Strategy", array.array, typecode="d")
    
    ## définition de la toolbox
    toolbox = base.Toolbox()
    toolbox.register("individual", generateES, creator.Individual, creator.Strategy, \
    IND_SIZE, MIN_VALUE, MAX_VALUE, MIN_STRATEGY, MAX_STRATEGY)
    toolbox.register("population", tools.initRepeat, list, toolbox.individual)
    toolbox.register("map",futures.map)
    toolbox.register("mate", tools.cxSimulatedBinaryBounded,
                     low=MIN_VALUE, up=MAX_VALUE, eta=20.0)
    toolbox.register("mutate", tools.mutPolynomialBounded,
                     low=MIN_VALUE, up=MAX_VALUE, eta=20.0, indpb=1.0 / IND_SIZE)
    toolbox.decorate("mate", checkStrategy(MIN_STRATEGY))
    toolbox.decorate("mutate", checkStrategy(MIN_STRATEGY))
    toolbox.register("evaluate", eval_nn, env=environment)
    toolbox.register("select", tools.selNSGA2)
    
    ## création de la population
    population = toolbox.population(n=mu)
    
    paretofront = tools.ParetoFront()
    
    # Evaluate the individuals with an invalid fitness
    invalid_ind = [ind for ind in population if not ind.fitness.valid]
    fitnesses_bds = toolbox.map(toolbox.evaluate, invalid_ind)
    
    for ind, (fit, bd) in zip(invalid_ind, fitnesses_bds):
        if (variant=="FIT+NS"):
            ind.fitness.values=(fit,0)
        elif (variant=="FIT"):
            ind.fitness.values=(fit,)
        elif (variant=="NS"):
            ind.fitness.values=(0,)
        ind.fit = fit
        ind.bd = bd
        
    population = toolbox.select(population, len(population))
    if paretofront is not None:
        paretofront.update(population)
    #print("Pareto Front: "+str(paretofront))

    k=15
    add_strategy="random"
    lambdaNov=6

    if variant=='NS' or variant=='FIT+NS':
    	archive=updateNovelty(population,population,None,k,add_strategy,lambdaNov)

    for ind in population:
        if (variant=="FIT+NS"):
            ind.fitness.values=(ind.fit,ind.novelty)
        elif (variant=="FIT"):
            ind.fitness.values=(ind.fit,)
        elif (variant=="NS"):
            ind.fitness.values=(ind.novelty,)
        
    # Begin the generational process
    for gen in range(1, ngen + 1):
        if (gen%10==0):
            print("+",end="", flush=True)
        else:
            print(".",end="", flush=True)

        ## générer un ensemble de points à partir de la population courante:
        offspring = algorithms.varOr(population, toolbox, lambda_, cxpb=cxpb, mutpb=mutpb)

        # Evaluate the individuals with an invalid fitness
        invalid_ind = [ind for ind in offspring if not ind.fitness.valid]
        fitnesses_bds = toolbox.map(toolbox.evaluate, invalid_ind)
        
        for ind, (fit, bd) in zip(invalid_ind, fitnesses_bds):
            if (variant=="FIT+NS"):
                ind.fitness.values=(fit,0)
            elif (variant=="FIT"):
                ind.fitness.values=(fit,)
            elif (variant=="NS"):
                ind.fitness.values=(0,)
            ind.fit = fit
            ind.bd = bd
       
        pq=population+offspring
        
        if variant=='NS' or variant=='FIT+NS':
        	archive=updateNovelty(pq,offspring,archive,k,add_strategy,lambdaNov)

        for ind in pq:
            if (variant=="FIT+NS"):
                ind.fitness.values=(ind.fit,ind.novelty)
            elif (variant=="FIT"):
                ind.fitness.values=(ind.fit,)
            elif (variant=="NS"):
                ind.fitness.values=(ind.novelty,)
           
        ## choisir la nouvelle population à partir de pq
        population = toolbox.select(pq,mu)
        # Update the hall of fame with the generated individuals
        if paretofront is not None:
            paretofront.update(population)

    indexmin, _ = min(enumerate([i.fit for i in paretofront]), key=operator.itemgetter(1))
    eval_nn(paretofront[indexmin], env=environment, name=variant, render=True)
          
    return population, paretofront, paretofront[indexmin]

