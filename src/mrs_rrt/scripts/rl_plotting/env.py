"""
Environment for rrt_2D
@author: huiming zhou
"""


class Env:
    def __init__(self):
        self.x_range = (-2, 10)
        self.y_range = (-7, 7)
        self.obs_boundary = self.obs_boundary_forest()
        self.obs_circle = self.obs_circle()
        self.obs_rectangle = self.obs_rectangle()


    
    @staticmethod
    def obs_boundary():
        obs_boundary = [
            [-2.3, -7.3, 0.3, 14.3],
            [-2.3, 7, 12.3, 0.3],
            [-2, -7.3, 12.3, 0.3],
            [10, -7, 0.3, 14.3]
       ]
        return obs_boundary

    @staticmethod
    def obs_boundary_forest():
        obs_boundary = [
            [-4.3, -7.3, 0.3, 14.3],
            [-4.3, 7, 14.3, 0.3],
            [-4, -7.3, 14.3, 0.3],
            [10, -7, 0.3, 14.3]
        ]
        return obs_boundary


    @staticmethod
    def obs_rectangle():
        obs_rectangle = [
        #    [1.85, -2, 0.3, 4],
        #    [1.85, 2, 4, 0.3],
        #    [1.85, -2.3, 4, 0.3]
        ]
        return obs_rectangle

    @staticmethod
    def obs_circle():
        obs_cir = [            
            [-1, 0, 0.25],
            [2, 0, 0.25],
            [0, 2.5, 0.25],
            [0.5, -2, 0.25],
            [3, 0, 0.25],
            [2, -3, 0.25],
            [6, 0, 0.25],
            [1.5, 4, 0.25],
            [4.5, 2, 0.25]
        ]

        return obs_cir
