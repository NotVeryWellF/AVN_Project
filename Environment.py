class Vehicle:
    """ Base vehicle class """
    def __init__(self, x: float, y: float, vel: float, goals: list):
        assert vel >= 0, "Velocity must be greater or equal to 0"
        self.x = x  # m
        self.y = y  # m
        self.vel = vel  # m/s
        # List goals of velocities by pairs: (x_coord, goal_velocity), goals must be in ascending order by x_coord
        self.goals = goals
        self.curr_goal = self.find_curr_goal()  # current desired velocity
        # Acceleration to reach the desired goal in time
        self.acc = (self.curr_goal[1]**2 - self.vel**2)/(2*(self.curr_goal[0] - self.x))

    def find_curr_goal(self):
        # if there is a goal on the list, with x_coord > self.x -> current goal velocity = goal's velocity,
        # in other cases goal velocity is 16 m/s
        for goal in self.goals:
            if self.x < goal[0]:
                return goal
        return 1000, 16


class Sender(Vehicle):
    """ Vehicle that can offload tasks """
    pass


class Receiver(Vehicle):
    """ Vehicle that can receive offloaded tasks """
    pass


class RSU:
    """ RSU Agent that controls the actions in the environment """

    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y
        self.radius = 250  # max radius of the connection


class Environment:
    """ Environment class """

    def __init__(self, _RSU: RSU, _Receiver: Vehicle, _Sender: Vehicle, step_size: float):
        # RSU of the area
        self._RSU = _RSU

        # Vehicle that can receive the offloading tasks
        self._Receiver = _Receiver

        # Vehicle that can send the offloading task to either RSU or Receiver
        self._Sender = _Sender

        # Size of the step in seconds
        self.step_size = step_size

    def renew_positions(self):
        # Updates the positions of the vehicles

        # Sender
        if self._Sender.x > self._Sender.curr_goal[0]:
            # find a new goal if old was passed
            self._Sender.curr_goal = self._Sender.find_curr_goal()
            # calculate the acceleration needed to reach the desired velocity in time
            self._Sender.acc = (self._Sender.curr_goal[1] ** 2 - self._Sender.vel ** 2) / \
                               (2 * (self._Sender.curr_goal[0] - self._Sender.x))
        # increment velocity and position of teh vehicle
        self._Sender.vel += self.step_size*self._Sender.acc
        self._Sender.x += self.step_size*self._Sender.vel

        # Receiver
        if self._Receiver.x > self._Receiver.curr_goal[0]:
            # find a new goal if old was passed
            self._Receiver.curr_goal = self._Receiver.find_curr_goal()
            # calculate the acceleration needed to reach the desired velocity in time
            self._Receiver.acc = (self._Receiver.curr_goal[1] ** 2 - self._Receiver.vel ** 2) / \
                               (2 * (self._Receiver.curr_goal[0] - self._Receiver.x))
        # increment velocity and position of teh vehicle
        self._Receiver.vel += self.step_size*self._Receiver.acc
        self._Receiver.x += self.step_size*self._Receiver.vel

