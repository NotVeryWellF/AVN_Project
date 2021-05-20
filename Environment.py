import numpy as np
import math
import random

from typing import Tuple


class Vehicle:
    """ Base vehicle class """

    # x - start x position of the vehicle (meters)
    # vel - start velocity of the vehicle (in positive x direction) (meters/second)
    # goals - list of speed limits (x, v), where x - start of the limit, v - max speed
    # T - time of the simulation (seconds), time_step - duration of one calculation step (seconds)
    def __init__(self, x: float, vel: float, goals: list, T: int, time_step: float):
        goals.append((10**6, 16))  # add final limit in the long distance
        N = math.ceil(T/time_step)  # Number of steps
        self.route = np.zeros((N,))  # Position of the vehicle in each point in time
        curr_goal_index = 0  # Current speed limit
        acc = (goals[curr_goal_index][1] ** 2 - vel ** 2) / (2 * (goals[curr_goal_index][0] - x))  # Start acceleration

        # Calculate route of the vehicle (position in each point of time)
        for i in range(N):
            if x > goals[curr_goal_index][0]:  # If vehicle reached the speed limit, it looks at the next one
                curr_goal_index += 1
                acc = (goals[curr_goal_index][1] ** 2 - vel ** 2) / (2 * (goals[curr_goal_index][0] - x))
            vel += acc*time_step
            x += vel*time_step
            self.route[i] = x


class Sender(Vehicle):
    """ Vehicle that can offload tasks """
    pass


class Receiver(Vehicle):
    """ Vehicle that can receive offloaded tasks """
    pass


class RSU:
    """ RSU Agent that controls the offloading decisions"""

    def __init__(self, x: float, y: float, radius: float):
        self.x = x
        self.y = y
        self.radius = radius  # max radius of the connection


class Environment:
    """ Environment class """

    def __init__(self, T: int, time_step: float, goal_number_range: Tuple[int, int],
                 goal_speed_range: Tuple[float, float], max_start_pos: float):
        # RSU of the area
        rsu_x_pos = 500
        rsu_y_pos = 5
        rsu_radius = 500
        self.rsu = RSU(rsu_x_pos, rsu_y_pos, rsu_radius)

        # Goals
        goals = self.generate_goals(goal_number_range, goal_speed_range, max_start_pos, rsu_radius*2)

        # Vehicle that can receive the offloading tasks
        receiver_pos = random.random()*max_start_pos
        receiver_vel = random.uniform(goal_speed_range[0], goal_speed_range[1])
        self.receiver = Receiver(receiver_pos, receiver_vel, goals, T, time_step)

        # Vehicle that can send the offloading task to either RSU or Receiver
        sender_pos = random.random() * max_start_pos
        sender_vel = random.uniform(goal_speed_range[0], goal_speed_range[1])
        self.sender = Sender(sender_pos, sender_vel, goals, T, time_step)

        # Environment information
        self.T = T
        self.time_step = time_step
        self.delta_distance = np.abs(self.sender.route - self.receiver.route)

    @staticmethod
    def generate_goals(goal_number_range: Tuple[int, int], goal_speed_range: Tuple[float, float],
                       max_start_pos: float, max_final_pos: float):
        return [(x, random.uniform(goal_speed_range[0], goal_speed_range[1])) for x in
                np.linspace(max_start_pos, max_final_pos, random.randint(goal_number_range[0], goal_number_range[1]))]




