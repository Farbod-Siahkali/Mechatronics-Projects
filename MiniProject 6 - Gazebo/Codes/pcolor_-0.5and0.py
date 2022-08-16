import numpy as np
import math
import matplotlib.pyplot as plt
#route to 1.7, 0
goal = [-0.5, 0]

x = np.linspace(-2, 2, 80)
y = np.linspace(-2, 2, 80)
goal_potential = np.zeros((80,80))
blocks_potential = np.zeros((80,80))
potential = np.zeros((80,80))

alpha = 5
beta = 100
s = 0.4

obstacles = [[-1.1, -1.1], [-1.1, 0], [-1.1, 1.1] , [0, -1.1], [0,0] , [0, 1.1], [1.1, -1.1], [1.1, 0], [1.1, 1.1]]

r = 0.15
l = 0.5

for i, xx in enumerate(x):
    for j, yy in enumerate(y):
        for ob in obstacles:
            goal_potential[i][j] = 0.5*alpha*math.sqrt(pow(xx-goal[0],2) + pow(yy-goal[1],2))
            if (math.sqrt(pow(xx-ob[0],2) + pow(yy-ob[1],2)))<r:
                blocks_potential[i][j] = 25
            
            elif (math.sqrt(pow(xx-ob[0],2) + pow(yy-ob[1],2)))>s:
                blocks_potential[i][j] += 0
            
            else:
                blocks_potential[i][j] +=  0.5*beta*(s+r-math.sqrt(pow(xx-ob[0],2) + pow(yy-ob[1],2)))

potential = blocks_potential+goal_potential

plt.figure()
plt.pcolor(potential)
plt.colorbar()
plt.show()

pos_now = (6, 40)
orientation = 0

from itertools import product

def neighbours(cell):
    for c in product(*(range(n-1, n+2) for n in cell)):
        if c != cell and all(0 <= n < 80 for n in c):
            yield c

path = []
path.append(pos_now)
path.append(pos_now)

while (pos_now != (30,40)):
 
    nears = list(neighbours((pos_now[0],pos_now[1])))
    flag = 0

    for near in nears:
        if flag == 0:
            temp = potential[near[0]][near[1]]
            temp_index = near
            flag = 1
        
        else:
            if potential[near[0]][near[1]] <= temp:
                if near == path[-2]:
                    continue
                temp = potential[near[0]][near[1]]
                temp_index = near

    path.append(temp_index)
    pos_now = temp_index

path.pop(0)

for p in path:
    potential[p[0]][p[1]] = 40

plt.figure()
plt.pcolor(potential)
plt.colorbar()
plt.show()

print("founded a route")