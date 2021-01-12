import numpy as np
from intersect import intersection

class Lidar:
    def __init__(self, samples, length, min_angle, max_angle):
        self.samples = samples
        self.length = length
        self.min_angle = min_angle
        self.max_angle = max_angle

    def laser_beam(self, body_position):
        delta_r = np.linspace(0, self.length, 100)
        interval = (np.pi * 2)/self.samples
        for i in range(self.samples):
            phi = -np.pi + i * interval
            x = delta_r * np.cos(phi) + body_position[0]
            y = delta_r * np.sin(phi) + body_position[1]
            beam[i] = [x, y] # 32 laser ray
        return beam


        x, y = intersection(x1, y1, x2, y2)

