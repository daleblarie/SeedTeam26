from FomationOptimization import INDIVIDUAL
import constants as c
import datetime
import copy
import random
import json

class POPULATION:
    def __init__(self, popSize):
        self.p = {}
        self.popSize = popSize

    def Print(self):
        for i in self.p:
            if (i in self.p):
                self.p[i].Print()
        print()

    def Initilize(self, initialK=None):
        for i in range(self.popSize):
            if i == 0:
                self.p[i] = INDIVIDUAL(i, k=initialK)
            self.p[i] = INDIVIDUAL(i)

    def Evaluate(self,pb):
        for i in self.p:
            self.p[i].Start_Evaluation(pb)

        for i in self.p:
            self.p[i].Compute_Fitness()

    def Mutate(self):
        for i in self.p:
            self.p[i].Mutate()

    def ReplaceWith(self, other):
        for i in self.p:
            if self.p[i].fitness < other.p[i].fitness:
                self.p[i] = other.p[i]

    def Fill_From(self, other):
        self.Copy_Best_From(other)
        self.Collect_Children_From(other)

        for i in self.p:
            self.p[i].fitness = 0

    def Copy_Best_From(self, other):
        fitnesses = [other.p[i].fitness for i in other.p]
        best = fitnesses.index(max(fitnesses))

        # print(other.p[best].k)

        self.p[0] = other.p[best]

    def Collect_Children_From(self, other):
        for i in list(other.p.keys())[1:]:
            winner = other.Winner_Of_Tournament_Selection()
            self.p[i] = copy.deepcopy(winner)
            self.p[i].Mutate()


    def Winner_Of_Tournament_Selection(other):
        p1 = random.randint(0, len(other.p)-1)

        p2 = random.randint(0, len(other.p)-1)

        while p1 == p2:
            p2 = random.randint(0, len(other.p) - 1)

        if other.p[p1].fitness > other.p[p2].fitness:
            return other.p[p1]
        else:
            return other.p[p2]

    def showBest(self, save=False):
        fitnesses = [self.p[i].fitness for i in self.p]
        best = fitnesses.index(max(fitnesses))

        # self.p[best].Start_Evaluation(False, pause=True)
        self.p[best].Start_Evaluation(pb=False, save=save)

        if save:
            fileName = 'Formation_bestC0nstants.txt'
            json_file = open(fileName, 'a')
            txt = {'TimeStamp': str(datetime.datetime.now()),
                   'k': self.p[best].k,
                   'Fitness': self.p[best].fitness}
                   # 'simInfor': {'numGens': c.numGens,
                   #              'popSize': c.popSize,
                   #              'N': c.N,
                   #              'timeSteps': c.timeSteps,
                   #              'target': c.target}}
            json.dump(txt, json_file)
            json_file.write('\n')
            json_file.close()
