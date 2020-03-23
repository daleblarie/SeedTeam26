from population import POPULATION
import constants as c
import copy
import pickle
import matplotlib.pyplot as plt

parents = POPULATION(c.popSize)
parents.Initilize()
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
