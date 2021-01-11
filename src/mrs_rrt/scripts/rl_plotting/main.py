import os
import sys
import math
import numpy as np
import pickle


sys.path.append(os.path.dirname(os.path.abspath(__file__)))
import env, plotting

outdir = './mrs_pp/src/mrs_rrt/scripts/rl_plotting/'
os.chdir(outdir)

print(os.getcwd())

file_name = "rl_path"
open_file = open(file_name, "rb")
path = pickle.load(open_file)
open_file.close()

s_start = [0, 4]
s_goal = [4, -4]
plotter = plotting.Plotting(s_start, s_goal)

#plotter.animation(rrt.vertex, path, "PPO", True)
plotter.animation([], path, "PPO", True)