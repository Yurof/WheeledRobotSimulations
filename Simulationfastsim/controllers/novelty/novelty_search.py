from scipy.spatial import KDTree, distance
import random
import numpy as np


class NovArchive:
    """Archive used to compute novelty scores."""
    def __init__(self, lbd, k=15):
        self.all_bd=lbd
        self.kdtree=KDTree(self.all_bd)
        self.k=k
        #print("Archive constructor. size = %d"%(len(self.all_bd)))
        
    def update(self,new_bd):
        oldsize=len(self.all_bd)
        self.all_bd=self.all_bd + new_bd
        self.kdtree=KDTree(self.all_bd)
        #print("Archive updated, old size = %d, new size = %d"%(oldsize,len(self.all_bd)))
        
    def get_nov(self,bd, population=[]):

        ## à faire: calcul de la nouveauté: somme de la distance dans l'espace comportemental aux k plus proches voisins parmi la population et l'archive des
        ## individus déjà explorés. Le descripteur comportemental par rapport auquel calculer toutes les distances est bd. ind.bd permet de récupérer le
        ## descripteur d'un individu ind de la population. Concernant l'archive, utilisez la fonction self.kdtree.query.
        pop_bds = [ind.bd for ind in population]
        pop_tree = KDTree(pop_bds)
      
        pop_dist = pop_tree.query(bd, self.k)[0]
        archive_dist = self.kdtree.query(bd, self.k)[0]
        
        return np.sum(np.sort(np.append(pop_dist, archive_dist))[:self.k])

    def size(self):
        return len(self.all_bd)
    
def updateNovelty(population, offspring, archive, k=15, add_strategy="random", _lambda=6, verbose=False):
   """Update the novelty criterion (including archive update) 

   Implementation of novelty search following (Gomes, J., Mariano, P., & Christensen, A. L. (2015, July). Devising effective novelty search algorithms: A comprehensive empirical study. In Proceedings of GECCO 2015 (pp. 943-950). ACM.).
   :param population: is the set of indiv for which novelty needs to be computed
   :param offspring: is the set of new individuals that need to be taken into account to update the archive (may be the same as population, but it may also be different as population may contain the set of parents)
   :param k: is the number of nearest neighbors taken into account
   :param add_strategy: is either "random" (a random set of indiv is added to the archive) or "novel" (only the most novel individuals are added to the archive).
   :param _lambda: is the number of individuals added to the archive for each generation
   The default values correspond to the one giving the better results in the above mentionned paper.

   The function returns the new archive
   """
   
   # Novelty scores updates
   if (archive) and (archive.size()>=k):
       if (verbose):
           print("Update Novelty. Archive size=%d"%(archive.size())) 
       for ind in population:
           ind.novelty=archive.get_nov(ind.bd, population)
   else:
       if (verbose):
           print("Update Novelty. Initial step...") 
       for ind in population:
           ind.novelty=0.

   if (verbose):
       print("Fitness (novelty): ",end="") 
       for ind in population:
           print("%.2f, "%(ind.novelty),end="")
       print("")
   if (len(offspring)<_lambda):
       print("ERROR: updateNovelty, lambda(%d)<offspring size (%d)"%(_lambda, len(offspring)))
       return None

   lbd=[]
   # Update of the archive
   if(add_strategy=="random"):
       l=list(range(len(offspring)))
       random.shuffle(l)
       if (verbose):
           print("Random archive update. Adding offspring: "+str(l[:_lambda])) 
       lbd=[offspring[l[i]].bd for i in range(_lambda)]
   elif(add_strategy=="novel"):
       soff=sorted(offspring,lambda x:x.novelty)
       ilast=len(offspring)-_lambda
       lbd=[soff[i].bd for i in range(ilast,len(soff))]
       if (verbose):
           print("Novel archive update. Adding offspring: ")
           for offs in soff[iLast:len(soff)]:
               print("    nov="+str(offs.novelty)+" fit="+str(offs.fitness.values)+" bd="+str(offs.bd))
   else:
       print("ERROR: updateNovelty: unknown add strategy(%s), valid alternatives are \"random\" and \"novel\""%(add_strategy))
       return None
       
   if(archive==None):
       archive=NovArchive(lbd,k)
   else:
       archive.update(lbd)

   return archive
