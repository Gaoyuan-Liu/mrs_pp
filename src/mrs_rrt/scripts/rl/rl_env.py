"""
RL Environment
@author: Gaoyuan Liu
"""

import os
import sys
import math
import numpy as np

import gym
from gym.utils import seeding
from gym.envs.registration import register
from gym import spaces

sys.path.append(os.path.dirname(os.path.abspath(__file__)))

import env, plotting, utils
import matplotlib.pyplot as plt



class Node:
    def __init__(self, n):
        self.x = n[0]
        self.y = n[1]
        self.parent = None

reg = register(
    id='RLPP-v0',
    entry_point='rl_env:PathPlanningEnv',
    max_episode_steps=30,
    )

class PathPlanningEnv(gym.Env):
    def __init__(self):
        super(PathPlanningEnv, self).__init__()

        self.utils = utils.Utils()
        

        self.action_space = spaces.Box(low=np.array([-1, -1]), high=np.array([1, 1]), dtype=np.float32)
        self.observation_space = spaces.Box(low=np.array([-4, -7]), high=np.array([10, 7]), dtype=np.float32)

        self.start_position = np.array([0, 4])
        self.goal_position = np.array([4, -4])
        self.plotter = plotting.Plotting(self.start_position, self.goal_position)
        self.current_position = self.start_position
        self.prev_state = self.current_position

        self.step_count = 0
        self.episode_length = 40
        self.episode_count = 0
        #self.episode_ = 0

        self.path = [self.start_position]
        self.nodelist = [Node(self.start_position)]
        self.reached_godal = False

    def reset(self):
        self.current_position = self.start_position
        self.prev_state = self.current_position
        self.step_count = 0

        self.current_position = self.start_position
        self.episode_count += 1

        if self.reached_godal == False:
            self.path = []

        plt.close()
        self.plotter.animation(self.nodelist, self.path, "n_episode" + str(self.episode_count), True)
        plt.pause(0.5)
        
        self.path = [self.start_position]
        self.nodelist = [Node(self.start_position)]
        observation = self.current_position
        self.reached_godal = False
        return observation
        

    def step(self, action):
        wind_noise = np.random.rand(2) * 0.5
        action = action

        current_node = Node(self.current_position)
 
        next_posistion = np.array(self.current_position) + action
        next_node = Node(next_posistion)

        reward_os = 0
        if self.utils.is_collision(current_node, next_node):
            reward_os = -1
        else:
            self.current_position = next_posistion

        
        
        reward_dis, done = self.process_data(self.current_position)
        reward = reward_os + reward_dis

        state = self.current_position
        node = Node(state)
        node.parent = Node(self.prev_state)
        
        self.prev_state = state
        self.path.append(state)
        
        self.nodelist.append(node)
        #self.plotter.animation([], self.path, "PPO", True)

        self.step_count += 1
        
        
        if self.step_count >= self.episode_length:
            done = True
        return state, reward, done, {}


        
    def process_data(self, current_position):
        dist = self.distance(current_position, self.goal_position)
        reward = 0
        done = False

        if dist < 1:
            reward += 5
            done = True
            self.path.append(np.array([4, -4]))
            self.reached_godal = True

        forward = self.distance(self.prev_state, self.goal_position) - self.distance(current_position, self.goal_position)
        if forward <= 0:
            reward += 2 * forward
        else:
            reward += forward
        #print(forward)    
        return reward, done




    def distance(self, position_1, position_2):
        err = np.subtract(position_1, position_2)
        dist = np.linalg.norm(err)
        return dist


    def lidar(self):
        n_beam = 32
        radius = 3
        




