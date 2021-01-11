import numpy as np
from rl_env import Node 
a = [3,3,3]
b = [1,2,1]
c = np.subtract(a,b)
print(np.linalg.norm(c))