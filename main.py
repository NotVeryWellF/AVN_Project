from Environment import *
from random import randint
import numpy as np
import math
import matplotlib.pyplot as plt

T = 60  # time of simulation in seconds
time_step = 0.001  # time of each step of calculations in seconds
N = math.ceil(T/time_step)  # Number of steps
time = np.linspace(0, T, N)  # Time steps
goal_number_range = (3, 7)  # Range of the number of speed limits on the road
goal_speed_range = (10, 20)  # Range of the speed of vehicles
task_number_range = (1, 5)  # Range of the number of tasks
max_start_pos = 50.  # Maximum starting x position of the vehicles


# Generate new environment
env = Environment(T, time_step, goal_number_range, goal_speed_range, max_start_pos, task_number_range)
env.simulate()

# fig, axs = plt.subplots(2, 2)
# axs[0, 0].plot(time, receiver_pos)
# axs[0, 0].plot(time, sender_pos)
# axs[0, 1].plot(time, receiver_vel)
# axs[0, 1].plot(time, sender_vel)
# axs[1, 0].plot(time, receiver_acc)
# axs[1, 0].plot(time, sender_acc)
# axs[1, 1].plot(time, np.abs(receiver_pos - sender_pos))

fig, axs = plt.subplots(2, 2)
axs[0, 0].plot(time, env.sender.route)
axs[0, 0].plot(time, env.receiver.route)
axs[0, 1].plot(time, env.delta_distance)
plt.show()
