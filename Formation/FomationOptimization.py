from Formation import FORMATION
from simulation import SIMULATION
import constants as c
import numpy as np
import random
import math


class INDIVIDUAL:
    def __init__(self, i):
        self.ID = i
        self.k = {0: [0, 0], 1: [0, 0], 2: [0, 0, 0], 3: [0, 0, 0], 4: [0, 0, 0]}
        for i in self.k:
            self.k[i] = list(np.random.rand(len(self.k[i])) * 2 - 1)

        self.fitness = 0
        self.sim = None
        self.form = None

    def Start_Evaluation(self, pb=True, save=False):
        target = np.zeros((6, 1))
        target[0, 0] = 300
        target[1, 0] = 0
        target[4, 0] = 2
        # theta = 2*math.pi/3
        # theta = math.radians(36.87 * 2)
        N = c.N
        # initialPos = np.random.randint(-10, 10, (2, N))
        initialPos = np.asarray([[0, -20, 20, 0, 0], [0, 0, 0, 20, -20]])

        self.form = FORMATION(N, initialPos, target, k=self.k)

        # theta = self.form.theta_min()
        # self.form.V_Formation(theta, target)
        # self.form.circular_formation()

        # form.nodal_input()
        self.sim = SIMULATION(self.form, c.timeSteps)

        self.sim.run(pb=pb, save=save)

    def Compute_Fitness(self):
        node_close = np.where(self.sim.node_dist < self.form.r_a)
        # node_close = np.argmin(self.sim.node_dist, axis=0)
        # nearNode = sum(self.sim.node_dist[node_close[0], node_close[1]]) / len(node_close[0])
        nearNode = len(node_close[0])

        # dist_mask = np.where(self.sim.target_dist > self.form.r_tau)
        # nearTarget = sum(np.transpose(self.sim.target_dist))/len(self.sim.target_dist)
        nearTarget = sum(self.sim.target_dist[:, -1] - self.form.r_tau)/len(self.sim.target_dist[0, :])

        too_close = np.where(self.sim.agent_dist < self.form.r_r)
        # too_far = np.where(self.sim.agent_dist > 100)
        # nearAgent = sum(self.sim.agent_dist[too_close]) /len(too_close)
        nearAgent = len(too_close[0])# + len(too_far[0])

        self.fitness = ((nearNode) - (nearAgent)) + (1 - nearTarget)
        # self.fitness = (1 - nearTarget)

        del self.form

    def Mutate(self):
        geneToMutate_i = random.randint(0, len(self.k)-1)
        geneToMutate_j = random.randint(0, len(self.k[geneToMutate_i])-1)
        self.k[geneToMutate_i][geneToMutate_j] = random.gauss(self.k[geneToMutate_i][geneToMutate_j], math.fabs(self.k[geneToMutate_i][geneToMutate_j]))


    def Print(self):
        print('[ ', end=''),
        print(self.ID, end=': '),
        print(self.fitness, end=' ')
        print('] ', end='')