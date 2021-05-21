import numpy as np
import math
import random

from typing import Tuple, List


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


class Task:
    """Task class"""
    def __init__(self, task_size: float, time: int, task_deadline: float, required_cycles: float):
        self.task_size = task_size  # size of task (Mb)
        self.output_size = self.task_size/100
        self.time = time
        self.task_deadline = task_deadline
        self.required_cycles = required_cycles  # task required cycles


class Sender(Vehicle):
    """ Vehicle that can offload tasks """
    def __init__(self, x: float, vel: float, goals: list, T: int, time_step: float, tasks: List[Task]):
        super().__init__(x, vel, goals, T, time_step)
        self.tasks = tasks  # all tasks


class Receiver(Vehicle):
    """ Vehicle that can receive offloaded tasks """
    def __init__(self, x: float, vel: float, goals: list, T: int, time_step: float, computation_resource: float):
        super().__init__(x, vel, goals, T, time_step)
        self.computation_resource = computation_resource  # computation resources for the offloading

    def get_execution_delay(self, task_size: float, required_cycles: float):
        return 0.01*task_size*required_cycles/self.computation_resource


class RSU:
    """ RSU Agent that controls the offloading decisions"""

    def __init__(self, x: float, y: float, radius: float, computation_resource: float):
        self.x = x
        self.y = y
        self.radius = radius  # max radius of the connection
        self.computation_resource = computation_resource  # computation resources for the offloading

    def get_execution_delay(self, task_size: float, required_cycles: float):
        return 0.01*task_size*required_cycles/self.computation_resource


class Communication:
    """Base Communication class"""
    def __init__(self, uplink_transmit_power: float, downlink_transmit_power: float, uplink_power_gain: float,
                 downlink_power_gain: float, noise: float, path_loss_exp: int, bandwidth: float):
        self.uplink_transmit_power = uplink_transmit_power
        self.downlink_transmit_power = downlink_transmit_power
        self.uplink_power_gain = uplink_power_gain
        self.downlink_power_gain = downlink_power_gain
        self.noise = noise
        self.path_loss_exp = path_loss_exp
        self.bandwidth = bandwidth

    def get_uplink_sinr(self, delta_distance: float):
        return (self.uplink_transmit_power * self.uplink_power_gain * delta_distance ** (-self.path_loss_exp)) / self.noise

    def get_downlink_sinr(self, delta_distance: float):
        return (self.downlink_transmit_power * self.downlink_power_gain * delta_distance ** (-self.path_loss_exp)) / self.noise

    def get_uplink_achievable_spectrum_efficiency(self, delta_distance: float):
        return self.bandwidth*math.log2(1 + self.get_uplink_sinr(delta_distance))

    def get_downlink_achievable_spectrum_efficiency(self, delta_distance: float):
        return self.bandwidth*math.log2(1 + self.get_downlink_sinr(delta_distance))

    def get_uplink_transmission_delay(self, task_size: float, delta_distance: float):
        return task_size/self.get_uplink_achievable_spectrum_efficiency(delta_distance)

    def get_downlink_transmission_delay(self, output_size: float, delta_distance: float):
        return output_size/self.get_downlink_achievable_spectrum_efficiency(delta_distance)


class V2VCommunication(Communication):
    """V2V Communication"""
    pass


class V2ICommunication(Communication):
    """V2I Communication"""
    def __init__(self, uplink_transmit_power: float, downlink_transmit_power: float, uplink_power_gain: float,
                 downlink_power_gain: float, noise: float, path_loss_exp: int, bandwidth: float,
                 optical_link_capacity: float):
        super().__init__(uplink_transmit_power, downlink_transmit_power, uplink_power_gain, downlink_power_gain, noise,
                         path_loss_exp, bandwidth)
        self.optical_link_capacity = optical_link_capacity  # communication link between home RSU and neighbor RSU

    def get_uplink_transmission_delay_to_neighbor(self, task_size: float):
        return task_size/self.optical_link_capacity

    def get_downlink_transmission_delay_to_neighbor(self, output_size: float):
        return output_size/self.optical_link_capacity


class Environment:
    """ Environment class """

    def __init__(self, T: int, time_step: float, goal_number_range: Tuple[int, int],
                 goal_speed_range: Tuple[float, float], max_start_pos: float, task_number_range: Tuple[int, int]):
        # Home RSU
        home_rsu_x_pos = 500
        home_rsu_y_pos = 5
        home_rsu_radius = 500
        home_rsu_computation_resource = 1000
        self.home_rsu = RSU(home_rsu_x_pos, home_rsu_y_pos, home_rsu_radius, home_rsu_computation_resource)

        # Neighboring RSU
        neighbor_rsu_x_pos = 700
        neighbor_rsu_y_pos = 300
        neighbor_rsu_radius = 500
        neighbor_rsu_computation_resource = 2500
        self.neighbor_rsu = RSU(neighbor_rsu_x_pos, neighbor_rsu_y_pos, neighbor_rsu_radius, neighbor_rsu_computation_resource)

        # Speed limits on the road
        goals = self.generate_goals(goal_number_range, goal_speed_range, max_start_pos, home_rsu_radius*2)

        # Vehicle that can receive the offloading tasks
        receiver_computation_resource = 250
        receiver_pos = random.random()*max_start_pos
        receiver_vel = random.uniform(goal_speed_range[0], goal_speed_range[1])
        self.receiver = Receiver(receiver_pos, receiver_vel, goals, T, time_step, receiver_computation_resource)

        # Vehicle that can send the offloading task to either RSU or Receiver
        sender_tasks = []
        task_number = random.randint(task_number_range[0], task_number_range[1])
        task_time = [int(x) for x in np.linspace(random.randint(2, 10), random.randint(T - 15, T - 5), task_number)]

        # Generate tasks
        for i in range(task_number):
            new_task_size = random.randint(100, 200)
            new_task_required_cycles = random.randint(100, 400)
            new_task_deadline = random.randint(1, 5)
            new_task = Task(new_task_size, task_time[i], new_task_deadline, new_task_required_cycles)
            sender_tasks.append(new_task)
        sender_pos = random.random() * max_start_pos
        sender_vel = random.uniform(goal_speed_range[0], goal_speed_range[1])
        self.sender = Sender(sender_pos, sender_vel, goals, T, time_step, sender_tasks)

        # V2V communication link
        v2v_uplink_transmit_power = 23
        v2v_downlink_transmit_power = 23
        v2v_uplink_power_gain = 4
        v2v_downlink_power_gain = 4
        v2v_noise = 174
        v2v_path_loss_exp = 4
        v2v_bandwidth = 2*10**7
        self.v2v = V2VCommunication(v2v_uplink_transmit_power, v2v_downlink_transmit_power, v2v_uplink_power_gain,
                                    v2v_downlink_power_gain, v2v_noise, v2v_path_loss_exp, v2v_bandwidth)

        # V2I communication link
        v2i_uplink_transmit_power = 23
        v2i_downlink_transmit_power = 23
        v2i_uplink_power_gain = 16
        v2i_downlink_power_gain = 16
        v2i_noise = 174
        v2i_path_loss_exp = 3
        v2i_bandwidth = 2 * 10 ** 7
        v2i_optical_link_capacity = 1000
        self.v2i = V2ICommunication(v2i_uplink_transmit_power, v2i_downlink_transmit_power, v2i_uplink_power_gain,
                                    v2i_downlink_power_gain, v2i_noise, v2i_path_loss_exp, v2i_bandwidth,
                                    v2i_optical_link_capacity)

        # Environment information
        self.T = T
        self.time_step = time_step
        self.delta_distance = np.abs(self.sender.route - self.receiver.route)
        self.N = math.ceil(self.T/self.time_step)

    # Simulates task offloading
    def simulate(self):
        # For each task
        for i in range(len(self.sender.tasks)):
            # Print parameters of the current task
            print("\nTask {}:\ntime: {}s\ndeadline: {}s\ntask size: {}Mb\ntask required cycles: {}\n".format(
                i + 1, self.sender.tasks[i].time, self.sender.tasks[i].task_deadline, self.sender.tasks[i].task_size,
                self.sender.tasks[i].required_cycles))
            task_step = math.ceil((self.sender.tasks[i].time / self.T) * self.N)

            # For V2V Communication
            print("For V2V Mode:")
            if self.sender.tasks[i].required_cycles <= self.receiver.computation_resource:
                total_delay = 0

                # Transmission delay
                uplink_transmission_delay = self.v2v.get_uplink_transmission_delay(self.sender.tasks[i].task_size,
                                                                                   self.delta_distance[task_step])
                print("Transmission delay: {}s".format(uplink_transmission_delay))
                total_delay += uplink_transmission_delay

                # Execution delay
                execution_delay = self.receiver.get_execution_delay(self.sender.tasks[i].task_size, self.sender.tasks[i].required_cycles)
                print("Execution delay: {}s".format(execution_delay))
                total_delay += execution_delay
                print("Total delay: {}s".format(total_delay))

                # If offloading is possible
                if total_delay < self.sender.tasks[i].task_deadline:
                    print("Task can be offloaded in this mode\n")
                else:
                    print("Task cannot be offloaded in this mode\n")
            else:
                print("Task requires too much computing resources.\nTask cannot be offloaded in this mode\n")

            # For V2I Communication
            print("For V2I Mode:")
            total_delay = 0

            # Transmission delay
            uplink_transmission_delay = self.v2i.get_uplink_transmission_delay(
                self.sender.tasks[i].task_size, math.hypot(abs(self.home_rsu.x - self.sender.route[task_step]),
                                                           self.home_rsu.y))
            print("Transmission delay: {}s".format(uplink_transmission_delay))
            total_delay += uplink_transmission_delay

            # Execution delay
            execution_delay = self.home_rsu.get_execution_delay(self.sender.tasks[i].task_size, self.sender.tasks[i].required_cycles)
            print("Execution delay: {}s".format(execution_delay))
            total_delay += execution_delay
            print("Total delay: {}s".format(total_delay))

            # If offloading is possible
            if total_delay < self.sender.tasks[i].task_deadline:
                print("Task can be offloaded in this mode\n")
            else:
                print("Task cannot be offloaded in this mode\n")

            # For V2I Communication with full offloading to the neighbor RSU
            print("For V2I Mode with full offloading to the neighbor RSU:")
            total_delay = 0

            # Transmission delay to the home RSU
            uplink_transmission_delay = self.v2i.get_uplink_transmission_delay(
                self.sender.tasks[i].task_size, math.hypot(abs(self.home_rsu.x - self.sender.route[task_step]),
                                                           self.home_rsu.y))
            print("Transmission delay to home RSU: {}s".format(uplink_transmission_delay))
            total_delay += uplink_transmission_delay

            # Transmission delay to the neighbor RSU
            uplink_transmission_delay = self.v2i.get_uplink_transmission_delay_to_neighbor(self.sender.tasks[i].task_size)
            print("Transmission delay to neighbor RSU: {}s".format(uplink_transmission_delay))
            total_delay += uplink_transmission_delay

            # Execution delay on the neighbor RSU
            execution_delay = self.neighbor_rsu.get_execution_delay(self.sender.tasks[i].task_size,
                                                                    self.sender.tasks[i].required_cycles)
            print("Execution delay: {}s".format(execution_delay))
            total_delay += execution_delay
            print("Total delay: {}s".format(total_delay))

            # If offloading is possible
            if total_delay < self.sender.tasks[i].task_deadline:
                print("Task can be offloaded in this mode\n")
            else:
                print("Task cannot be offloaded in this mode\n")

    # Method generates speed limits on the road
    @staticmethod
    def generate_goals(goal_number_range: Tuple[int, int], goal_speed_range: Tuple[float, float],
                       max_start_pos: float, max_final_pos: float):
        return [(x, random.uniform(goal_speed_range[0], goal_speed_range[1])) for x in
                np.linspace(max_start_pos, max_final_pos, random.randint(goal_number_range[0], goal_number_range[1]))]




