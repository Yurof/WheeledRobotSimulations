import pandas
import matplotlib.pyplot as plt
import numpy as np
 
def plot_violin(res):
    """ Makes a violin plot of the results provided in the argument

    Makes a violin plot of the results provided in the argument.
    :param res: dictionary of the results to plot. The key is the name and the data is a vector of performance values.
    """
    fig,ax=plt.subplots(figsize=(5,5))
    data=[]
    labels=[]
    for k in res.keys():
        data.append(res[k])
        labels.append(k)
    ax.violinplot(data,
                   showmeans=False,
                   showmedians=True)
    ax.set_title('Violin plot')
    # adding horizontal grid lines
    ax.yaxis.grid(True)
    ax.set_xticks([y + 1 for y in range(len(data))])
    ax.set_xlabel('Optimization methods')
    ax.set_ylabel('Mean squarred error')

    # add x-tick labels
    plt.setp(ax, xticks=[y + 1 for y in range(len(data))],
        xticklabels=labels)
    plt.setp(ax.get_xticklabels(), rotation=30, ha="right")
    plt.savefig("violin_plot", bbox_inches='tight', dpi=300)
    plt.show()
    

if __name__ == "__main__":
    
    # Dataset
    fitData = pandas.read_csv('results/individuals/Fitness/results.csv')
    fitData = fitData["mean_squared_error"].array
    
    novData = pandas.read_csv('results/individuals/NoveltySearch/results.csv')
    novData = novData["mean_squared_error"].array
    
    noveFitData = pandas.read_csv('results/individuals/NoveltyFitness/results.csv')
    noveFitData = noveFitData["mean_squared_error"].array
    
    data = [fitData, novData, noveFitData]
     
    data = {"fitness":fitData,"novelty":novData, "fitness + novelty":noveFitData}
    plot_violin(data)