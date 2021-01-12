import gym
import numpy as np

import random
import math
import time
import sys
import os

from numba import jit


# A map is a list of lists.
# - " " (space): open space, nothing
# - "x": a wall
# - a number: a goal. When the agent reaches the number, a reward of
number*10 is given to it and the episode terminates
#
# The map is normalized to a 1-by-1 square. Anywhere in this
environment, when a distance is given, it measures in "units", with one
unit the width of the map.
MAP_ROOMS = [
     'xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx',
     'x       x       x       x       x',
     'x       x       x       x       x',
     'x       x       x       x   1   x',
     'x       x       x       x       x',
     'x       x       x       x       x',
     'x       x       x       x       x',
     'x       x       x       x       x',
     'x       x       x       x       x',
     'xxxxxx xxxxxxx xxxxxxx xx       x',
     'x                               x',
     'x                               x',
     'x                               x',
     'xxxx xxxxxxx xxxxxxx xxxx       x',
     'x       x       x       x       x',
     'x       x       x       x       x',
     'x       x       x       x       x',
     'x       x       x       x       x',
     'x       x       x       x   2   x',
     'x       x       x       x       x',
     'x       x       x       x       x',
     'xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx',
]


MAP = MAP_ROOMS

SENSORS = 20
FIELD = 3.1415 * 0.66   # Field of view of the agent, here 120° (90*  ->
* 0.5)
RANGE = 1.0             # Sensor range in units. The grid is 1-by-1.
SPEED = 0.01            # How many units does the agent advance every
time-step.
HORIZON = 200           # Length of the episode in time-steps (the
episode finishes earlier if the agent finds the goal)

class VirtualOfficeEnv(gym.Env):
     """ Simulated room with walls
     """
     def __init__(self):
         # Load the map and transform it to a sequence of line segments
         wall_segments = []

         m = MAP
         self.box_h = 1. / len(m)
         self.box_w = 1. / len(m[0])
         known_segments = {}

         for y in range(len(m)):
             fy = y * self.box_h

             for x in range(len(m[0])):
                 fx = x * self.box_w

                 # (fx, fy) is the coordinate of the top-left corner of
the current cell
                 if m[y][x] != 'x':
                     # Not a wall
                     continue

                 # Look at whether there are walls around this block (in
which case
                 # the segments can be optimized)
                 left_wall = (x > 0 and m[y][x-1] == 'x')
                 top_wall = (y > 0 and m[y-1][x] == 'x')

                 segments = [
                     [fx, fy, fx, fy+self.box_h],
                     [fx, fy, fx+self.box_w, fy],
                     [fx, fy+self.box_h, fx+self.box_w, fy+self.box_h],
                     [fx+self.box_w, fy, fx+self.box_w, fy+self.box_h],
                 ]

                 if left_wall:
                     # Extend top and bottom segments, create right
segment, no
                     # need for a left segment.
                     left_segments = known_segments[(x-1, y)]
                     left_segments[1][2] = max(fx+self.box_w,
left_segments[1][2])           # New end x for top segment
                     left_segments[2][2] = fx+self.box_w
                      # New end x for bottom segment

                 if top_wall:
                     # Extend left and right segments of the top cell,
no need
                     # for a top segment, create bottom segment
                     top_segments = known_segments[(x, y-1)]
                     top_segments[0][3] = fy+self.box_h          # New
end y for left segment
                     top_segments[3][3] = fy+self.box_h          # New
end y for right segment

                 if left_wall and not top_wall:
                     # Share top, bottom and left segments with neighbor
                     segments[0] = left_segments[3]
                     segments[1] = left_segments[1]
                     segments[2] = left_segments[2]

                     wall_segments.append(segments[3])
                 elif top_wall and not left_wall:
                     # Share left, right and top with neighbor
                     segments[0] = top_segments[0]
                     segments[1] = top_segments[2]
                     segments[3] = top_segments[3]

                     wall_segments.append(segments[2])
                 elif top_wall and left_wall:
                     # Share all segments with the proper neighbor
                     segments[0] = top_segments[0]
                     segments[1] = left_segments[1]
                     segments[2] = left_segments[2]
                     segments[3] = top_segments[3]
                 else:
                     # Share no segment
                     wall_segments.extend(segments)


                 # Register the segments
                 known_segments[(x, y)] = segments

         # Configure Gym
         self.action_space = gym.spaces.Discrete(3)      # Left, right,
forward. Those are discrete actions!
         self.observation_space = gym.spaces.Box(
                 low=0.0, high=1.0,
                 shape=(SENSORS,)
             )

         self._timestep = 0
         self._episode = 0

         # Prepare Numpy arrays
         self._wall_segments = np.array(list(wall_segments),
dtype=np.float32)
         self._m = m

     def step(self, action, depth=1):
         """ Execute a continuous action (angle, speed)
         """
         self._timestep += 1

         # If the agent hits a wall, it does not move
         original_x = self._x
         original_y = self._y

         if action == 0 :
             # turn left
             self._angle = (self._angle - SPEED*10) % (2 * math.pi)
         elif action == 1 :
             # turn right
             self._angle = (self._angle + SPEED*10) % (2 * math.pi)
         elif action == 2 :
             # go forward
             self._x += math.cos(self._angle) * SPEED
             self._y += math.sin(self._angle) * SPEED

             self._x = max(0.0, min(self._x, 0.99))
             self._y = max(0.0, min(self._y, 0.99))

         reward = 0.0
         terminal = False

         # Check for collisions
         ix = int(self._x / self.box_w)
         iy = int(self._y / self.box_h)
         cell = self._m[iy][ix]

         if cell == 'x':
             reward = -1.0               # Reward of -1 if there is a
collision
             self._x = original_x
             self._y = original_y
         elif cell.isdigit():
             # Give a delayed reward
             reward = int(cell) * 10
             terminal = True

         # Return the current state, a reward and whether the episode
terminates
         return self.current_state(ix, iy), reward, terminal or
(self._timestep > HORIZON), {}

     def reset(self):
         """ Randomly put the agent in an empty cell of the map
         """
         self._timestep = 0
         self._episode += 1

         def clamp(x):
             return min(0.99, max(0.01, x))

         while True:
             # Try a random restart position
             fx = random.random()
             fy = random.random()

             # Try again if it is inside a wall
             x = int(fx / self.box_w)
             y = int(fy / self.box_h)

             if self._m[y][x] == 'x':
                 # Invalid location
                 continue
             else:
                 break

         self._x = fx
         self._y = fy
         self._angle = 6.28 * random.random()

         return self.current_state(x, y)

     def render(self):
         raise NotImplementedError('render() not implemented. Denis
Steckelmacher has a Python file that allows to visualize VirtualOffice,
if you want.')

     @staticmethod
     @jit(nopython=True, fastmath=True)
     def compute_distances(x1, y1, x2, y2, x3, y3, x4, y4):
         t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / ((x1 -
x2) * (y3 - y4) - (y1 - y2) * (x3 - x4))
         u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / ((x1 -
x2) * (y3 - y4) - (y1 - y2) * (x3 - x4))

         # When u is between 0 and 1, the intersection is in the sensor
segment, and u gives the distance reading
         # Only keep points that are on the sensor line, and on the
segment itself
         u[(u < 0.0) | (t < 0.0) | (t > 1.0)] = 1.0

         return u.min() / RANGE

     def current_state(self, ix, iy):
         """ Return the current state, a tuple of two channels
         """
         distances = []
         da = FIELD / SENSORS

         def f(x):
             return np.array([x], dtype=np.float32)

         x1 = self._wall_segments[:, 0]
         y1 = self._wall_segments[:, 1]
         x2 = self._wall_segments[:, 2]
         y2 = self._wall_segments[:, 3]
         x3 = f(self._x)
         y3 = f(self._y)

         for s in range(SENSORS):
             # Make a line segment for the sensor
             angle = self._angle - (FIELD / 2.) + da * s

             x4 = f(self._x + RANGE * math.cos(angle))
             y4 = f(self._y + RANGE * math.sin(angle))

             # Compute the intersections with all the obstacle segments
             distances.append(VirtualOfficeEnv.compute_distances(x1, y1,
x2, y2, x3, y3, x4, y4))

         self._last_distances = distances

         # Return state
         distances = np.array(distances, dtype=np.float32)
         distances = np.log(1. + distances)

         return distances