import os
import matplotlib.pyplot as plt
import numpy as np
#import RandomMouv_bullet

import csv

import pandas as pd

df = pd.read_csv('results/result.csv')


base_path = os.path.dirname(os.path.abspath(__file__))
img = plt.imread(f'{base_path}/models/scenes/maze_hard/maze_hard.pbm')

# plt.annotate("Goal", xy=(80, 320),  xytext=(65, 315))
# plt.plot(80, 320, 'go', markersize=25)

# plt.annotate("Start", xy=(160, 80),  xytext=(160, 60))
# plt.plot(160, 80, 'rx')

plt.annotate("Goal", xy=(20, 180),  xytext=(10, 170))
plt.plot(20, 180, 'go', markersize=25)

plt.annotate("Start", xy=(20, 40),  xytext=(20, 20))
plt.plot(20, 40, 'rx')

plt.imshow(np.flipud(img), origin='lower')

x = df.x
y = df.y
steps = df.steps
dist = df.distance_to_obj
plt.plot(x*img.shape[0]/10, y*img.shape[0]/10, label="test", alpha=0.7)
plt.axis('off')
plt.legend()
plt.show()


plt.plot(steps, dist)
plt.xlabel("steps")
plt.ylabel("distance de l'objectif")
plt.show()
