from Environment import *
from random import randint
import numpy as np
import math
import matplotlib.pyplot as plt

T = 30  # time of simulation in seconds
step_size = 0.01  # time of each step of calculations in seconds
N = math.ceil(T/step_size)  # number of steps needed


def random_env():
    number_of_goals = randint(3, 5)
    rsu = RSU(250, 0)
    sender_goals = [(x, randint(10, 25)) for x in np.linspace(50, 500, number_of_goals)]
    sender = Sender(randint(0, 30), randint(5, 10), randint(0, 25), sender_goals)
    receiver_goals = [(x, randint(10, 25)) for x in np.linspace(50, 500, number_of_goals)]
    receiver = Receiver(randint(0, 30), randint(5, 10), randint(0, 25), receiver_goals)
    return Environment(rsu, receiver, sender, step_size)


receiver_pos = np.zeros((N,))
sender_pos = np.zeros((N,))
receiver_vel = np.zeros((N,))
sender_vel = np.zeros((N,))
receiver_acc = np.zeros((N,))
sender_acc = np.zeros((N,))
time = np.linspace(0, T, N)

env = random_env()
print("Receiver:")
print(env._Receiver.x)
print(env._Receiver.vel)
print(env._Receiver.goals)
print("Sender:")
print(env._Sender.x)
print(env._Sender.vel)
print(env._Sender.goals)
for i in range(N):
    receiver_pos[i] = env._Receiver.x
    sender_pos[i] = env._Sender.x
    receiver_vel[i] = env._Receiver.vel
    sender_vel[i] = env._Sender.vel
    receiver_acc[i] = env._Receiver.acc
    sender_acc[i] = env._Sender.acc
    env.renew_positions()


fig, axs = plt.subplots(2, 2)
axs[0, 0].plot(time, receiver_pos)
axs[0, 0].plot(time, sender_pos)
axs[0, 1].plot(time, receiver_vel)
axs[0, 1].plot(time, sender_vel)
axs[1, 0].plot(time, receiver_acc)
axs[1, 0].plot(time, sender_acc)

plt.show()