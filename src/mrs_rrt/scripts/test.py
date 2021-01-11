import numpy as np
import pickle
import os
path = None
if path:
    print("I am here!")

a = np.array([1, 2, 3])
a.tolist()

print(type(a))
outdir = './mrs_rl/src/mrs_ppo/training_results'
os.chdir(outdir)
os.chdir('../../../../')

outdir = './mrs_pp/src/mrs_rrt/scripts/rl_plotting/'
os.chdir(outdir)
