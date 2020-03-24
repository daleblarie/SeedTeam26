import numpy as np

numGens = 100
popSize = 10
N = 5
timeSteps = 300

# 2D Formation
target = np.zeros((6, 1))
# initial target position
target[0, 0] = 200
target[1, 0] = 0
# initial target input
target[4, 0] = 2
