import numpy as np

numGens = 500
popSize = 20
N = 5
timeSteps = 100

# 2D Formation
target = np.zeros((6, 1))
# initial target position
target[0, 0] = 150
target[1, 0] = 0
# initial target input
target[4, 0] = 1
