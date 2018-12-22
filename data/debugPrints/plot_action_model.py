import matplotlib.pyplot as plt
import argparse
import numpy as np
import IPython
import math

parser = argparse.ArgumentParser(description='Plot the action model sampled distribution')
parser.add_argument('file', help='file with samples from action model as a csv (first row is origin point)')

args = parser.parse_args()

p = []
with open(args.file, 'r') as f:
    l = next(f)
    origin = [float(val) for val in l.split(',')]

    for l in f:
        pp = [float(val) for val in l.split(',')]
        p.append(pp)

# print(origin)
# print(p)

plt.figure()
plt.hold(True)
ax = plt.gca()
# fig, ax = plt.subplots()

l = 0.05
X = [pp[0] for pp in p]
Y = [pp[1] for pp in p]
U = [l*math.cos(pp[2]) for pp in p]
V = [l*math.sin(pp[2]) for pp in p]

# plt.scatter(origin[0], origin[1])
# plt.scatter([pp[0] for pp in p], [pp[1] for pp in p], alpha=0.2)
plt.axis('equal')
ax.quiver(origin[0], origin[1], l*math.cos(origin[2]), l*math.sin(origin[2]), units='xy', scale=1, color=[1,0.2,0.5])
ax.quiver(X, Y, U, V, units='xy', scale=1, color=[0,0.2,0.5,0.5])
# ax.quiverkey(q, X=0.3, Y=1.1, U=10,
#              label='Quiver key, length = 10', labelpos='E')


plt.xlabel('x (m)')
plt.ylabel('y (m)')
plt.legend(['origin', 'samples'])
plt.show()
