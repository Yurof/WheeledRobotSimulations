import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import
import numpy as np
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter
import random
from scipy.optimize import minimize
from deap import *

def plot_pareto_front(paretofront, title=""):
    x=[]
    y=[]
    for p in paretofront:
        print("Ind: "+str(p))
        fitness = p.fitness.values
        print("Fit: "+str(fitness))
        x.append(fitness[0])
        y.append(fitness[1])
    fig,ax=plt.subplots(figsize=(5,5))
    ax.plot(x,y,".")
    ax.set_title(title)
    plt.show()

def plot_pop_pareto_front(pop,paretofront, title=""):
    x=[]
    y=[]
    for p in paretofront:
        fitness = p.fitness.values
        x.append(fitness[0])
        y.append(fitness[1])
    xp=[]
    yp=[]
    for p in pop:
        fitness = p.fitness.values
        xp.append(fitness[0])
        yp.append(fitness[1])
    fig,ax=plt.subplots(figsize=(5,5))
    ax.plot(xp,yp,".", label="Population")
    ax.plot(x,y,".", label="Pareto Front")
    fitpareto=list(zip(x,y))
    fitpop=list(zip(xp,yp))
    print("Pareto: "+str(fitpareto))
    print("Population: "+str(fitpop))

    ax.set_title(title)
    plt.legend()
    plt.show()

