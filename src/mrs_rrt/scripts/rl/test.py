import numpy as np
from rl_env import Node
import utils
import matplotlib.pyplot as plt
#import intersection

from intersect import intersection

'''a, b = 1, 2
phi = np.linspace(3, 10, 100)
x1 = a*phi - b*np.sin(phi)
y1 = a - b*np.cos(phi)'''
phi = np.linspace(-np.pi, np.pi)
x1 = np.cos(phi) + 3
y1 = np.sin(phi) + 3

alpha = np.linspace(0, 3, 100)
x2 = 1 + alpha * np.cos(-np.pi/6)
y2 = alpha * np.sin(-np.pi/6)


x, y = intersection(x1, y1, x2, y2)

plt.plot(x1, y1, c="r")
plt.plot(x2, y2, c="g")
plt.plot(x, y, "*k")
plt.show()