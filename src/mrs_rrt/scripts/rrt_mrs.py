"""
RRT_2D
@author: huiming zhou
"""

import os
import sys
import math
import numpy as np
import rospy
from mrs_msgs.msg import UavState
from mrs_msgs.srv import Vec4

sys.path.append(os.path.dirname(os.path.abspath(__file__)))

#from Sampling_based_Planning.rrt_2D import env, plotting, utils
import env, plotting, utils



class Node:
    def __init__(self, n):
        self.x = n[0]
        self.y = n[1]
        self.parent = None


class Rrt:
    def __init__(self, s_start, s_goal, step_len, goal_sample_rate, iter_max):
        self.s_start = Node(s_start)
        self.s_goal = Node(s_goal)
        self.step_len = step_len
        self.goal_sample_rate = goal_sample_rate
        self.iter_max = iter_max
        self.vertex = [self.s_start]

        self.env = env.Env()
        self.plotting = plotting.Plotting(s_start, s_goal)
        self.utils = utils.Utils()

        self.x_range = self.env.x_range
        self.y_range = self.env.y_range
        self.obs_circle = self.env.obs_circle
        self.obs_rectangle = self.env.obs_rectangle
        self.obs_boundary = self.env.obs_boundary

    def planning(self):
        for i in range(self.iter_max):
            node_rand = self.generate_random_node(self.goal_sample_rate)
            node_near = self.nearest_neighbor(self.vertex, node_rand)
            node_new = self.new_state(node_near, node_rand)

            if node_new and not self.utils.is_collision(node_near, node_new):
                self.vertex.append(node_new)
                dist, _ = self.get_distance_and_angle(node_new, self.s_goal)

                if dist <= self.step_len and not self.utils.is_collision(node_new, self.s_goal):
                    self.new_state(node_new, self.s_goal)
                    return self.extract_path(node_new)

        return None

    def generate_random_node(self, goal_sample_rate):
        delta = self.utils.delta

        if np.random.random() > goal_sample_rate:
            return Node((np.random.uniform(self.x_range[0] + delta, self.x_range[1] - delta),
                         np.random.uniform(self.y_range[0] + delta, self.y_range[1] - delta)))

        return self.s_goal

    @staticmethod
    def nearest_neighbor(node_list, n):
        return node_list[int(np.argmin([math.hypot(nd.x - n.x, nd.y - n.y)
                                        for nd in node_list]))]

    def new_state(self, node_start, node_end):
        dist, theta = self.get_distance_and_angle(node_start, node_end)

        dist = min(self.step_len, dist)
        node_new = Node((node_start.x + dist * math.cos(theta),
                         node_start.y + dist * math.sin(theta)))
        node_new.parent = node_start

        return node_new

    def extract_path(self, node_end):
        path = [(self.s_goal.x, self.s_goal.y)]
        node_now = node_end

        while node_now.parent is not None:
            node_now = node_now.parent
            path.append((node_now.x, node_now.y))

        return path

    @staticmethod
    def get_distance_and_angle(node_start, node_end):
        dx = node_end.x - node_start.x
        dy = node_end.y - node_start.y
        return math.hypot(dx, dy), math.atan2(dy, dx)


def main():
    #x_start = (2, 2)  # Starting node
    #x_goal = (49, 24)  # Goal node
    x_start = (0, 4)  # Starting node
    x_goal = (4, -4)  # Goal node

    rrt = Rrt(x_start, x_goal, 0.5, 0.05, 10000)
    path = rrt.planning()

    if path:
        rrt.plotting.animation(rrt.vertex, path, "RRT", True)
    else:
        print("No Path Found!")

    return path


# Connection with mrs


class mrs_utils:
    def __init__(self):
        self.position_cmd_call = rospy.ServiceProxy('/uav1/control_manager/goto', Vec4)
        self.tolerable_error = 0.2

    def take_observation(self):
        data_pose = None
        
        while data_pose is None:
            
            try:
                data_uavstate = rospy.wait_for_message('/uav1/odometry/uav_state', UavState, timeout=1)
                data_pose = data_uavstate.pose 
            except:
                #a = 1
                rospy.loginfo("Current drone pose not ready yet, retrying for getting robot pose")

        return data_pose

    def distance(self, data_pose, reference_position):
        current_pose = [data_pose.position.x, data_pose.position.y, data_pose.position.z]
        
        err = np.subtract(current_pose, reference_position)
        #w = np.array([1, 1, 4])
        #err = np.multiply(w,err)
        dist = np.linalg.norm(err)
        return dist

    def cmd_achieve(self, command_call): 
        cmd_achieved = False
        cmd_position_3D = command_call[:-1]
        while cmd_achieved == False:
            data_pose = self.take_observation()
            dist = self.distance(data_pose, cmd_position_3D)
            if dist < self.tolerable_error:
                cmd_achieved = True
        return cmd_achieved



if __name__ == '__main__':
    path_reverse = main()
    path = path_reverse[::-1]
    print(path)

    mrs = mrs_utils()
    rospy.init_node('rrt_mrs_show', anonymous=True, disable_signals=True)
    
    at_start = True
    at_goal = False
    
    

    if at_start:
        for i in path:
            command_position = list(i)
            command_position.append(2.0)
            command_position.append(0.0)
            #print(command_position)
            data_pose = mrs.take_observation()
            
            mrs.position_cmd_call(command_position)
            cmd_achieved = mrs.cmd_achieve(command_position)


    if at_goal:
            mrs.position_cmd_call([4.0, -4.5, 2.0, 0.0])
            cmd_achieved = mrs.cmd_achieve([4.0, -4.5, 2.0, 0.0])
            mrs.position_cmd_call([-1.5, -4.5, 2.0, 0.0])
            cmd_achieved = mrs.cmd_achieve([-1.5, -4.5, 2.0, 0.0])
            mrs.position_cmd_call([-1.5, 4, 2.0, 0.0])
            cmd_achieved = mrs.cmd_achieve([-1.5, 4, 2.0, 0.0])
            mrs.position_cmd_call([0, 4, 2.0, 0.0])
            cmd_achieved = mrs.cmd_achieve([0, 4, 2.0, 0.0])
            


    