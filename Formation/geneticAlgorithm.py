from population import POPULATION
import constants as c
import copy
import pickle
import matplotlib.pyplot as plt
import json

fileName = 'Formation_bestC0nstants.txt'
with open(fileName, 'r') as json_file:
    last_line = json_file.readlines()[-1]
    k = json.loads(last_line)

k = {int(k): v for k, v in k['k'].items()}


parents = POPULATION(c.popSize)
parents.Initilize(k)
parents.Evaluate(pb=True)
# # exit()
print(0, end=' ')
parents.Print()

for i in range(c.numGens):
    children = POPULATION(c.popSize)
    children.Fill_From(parents)
    children.Evaluate(pb=True)
    print(i + 1, end=' ')
    children.Print()

    parents.ReplaceWith(children)

parents.showBest(save=True)


# Graph sensor data
#f = plt.figure()
#panel = f.add_subplot(111)
#panel.set_ylim(-1, +2)
#plt.plot(y)
#plt.show()
