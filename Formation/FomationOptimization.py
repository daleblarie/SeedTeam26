from Formation2D import FORMATION
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
        # theta = 2*math.pi/3
        # theta = math.radians(36.87 * 2)
        N = c.N
        # initialPos = np.random.randint(-10, 10, (2, N))
        # print(self.k)
        initialPos = np.asarray([[0, -20, 20, 0, 0], [0, 0, 0, 20, -20]])

        self.form = FORMATION(N, initialPos, c.target, k=self.k)

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
        nearNode = len(node_close[0])/self.sim.node_dist.shape[1]

        # all_active = np.where(self.sim.active_nodes == 1.0)
        # if len(all_active[0]) > 0:
        #     first_active = all_active[0][0]/c.timeSteps
        # else:
        #     first_ative = 0

        all_active = sum(self.sim.active_nodes[0, :])/c.timeSteps
        # print(self.form.nodes[6, :])

        end_active = sum(self.form.nodes[6, :])/c.N
        if end_active != 1:
            end_active = 0

        # if all_active == 1.0:
        #     all_active += 1
        # print(nearNode, all_active, self.sim.active_nodes[0, -1])

        # target_close = np.where((self.sim.target_dist - self.form.r_tau) > 5)
        diff = (self.sim.target_dist[0, :] - self.form.r_tau)
        # nearTarget = sum((diff - min(diff))/(max(diff) - min(diff)))/diff.shape[0]
        # nearTarget = len(target_close[0]) / self.sim.target_dist.shape[1]
        # nearTarget = sum(np.transpose(self.sim.target_dist))/len(self.sim.target_dist)
        nearTarget = self.sim.target_dist[0, -1]
        # print(nearTarget)

        # print(nearTarget, self.sim.target_dist[0, -1])

        agent_close = np.where(self.sim.agent_dist < self.form.r_r)
        # too_far = np.where(self.sim.agent_dist > 100)
        # nearAgent = sum(self.sim.agent_dist[too_close]) /len(too_close)
        nearAgent = ((len(agent_close[0]) - self.sim.agent_dist.shape[1])/(self.sim.agent_dist.size - self.sim.agent_dist.shape[1])) + 1

        if nearAgent == 0:
            nearAgent -= 1.0

        # print(nearAgent)
        # print('Form',self.sim.target_dist)
        # print(nearNode, all_active, self.sim.active_nodes[0, -1], nearAgent)

        # self.fitness = (nearNode)  * (all_active)  * (2 - nearAgent) + end_active#* (1 - nearTarget)
        self.fitness = (nearNode) * (all_active) * (1-abs(nearTarget-self.form.r_tau)) * (2 - nearAgent) + end_active
        # self.fitness = (1-abs(nearTarget-self.form.r_tau))

        del self.form
        del self.sim

    def Mutate(self):
        geneToMutate_i = random.randint(0, len(self.k)-1)
        geneToMutate_j = random.randint(0, len(self.k[geneToMutate_i])-1)
        self.k[geneToMutate_i][geneToMutate_j] = random.gauss(self.k[geneToMutate_i][geneToMutate_j], math.fabs(self.k[geneToMutate_i][geneToMutate_j]))


    def Print(self):
        print('[ ', end=''),
        print(self.ID, end=': '),
        print(self.fitness, end=' ')
        print('] ', end='')