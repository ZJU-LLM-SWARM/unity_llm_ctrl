import numpy as np


class VehicleModel:
    """
    Helper class that contains the parameters of the vehicle to be controlled

    Attributes:
        wheelbase: [m]
        max_speed: [m/s]
        max_acc: [m/ss]
        max_d_acc: [m/sss]
        max_steer: [rad]
        max_d_steer: [rad/s]
    """

    # def __init__(self):
    #     self.wheelbase = 0.3
    #     self.max_speed = 1.5
    #     self.max_acc = 2.0
    #     self.max_d_acc = 2.0
    #     self.max_steer = np.radians(45)
    #     self.max_d_steer = np.radians(45)
    def __init__(self):
        self.wheelbase = 1
        self.max_speed = 20.0
        self.max_acc = 8.0
        self.max_d_acc = 8.0
        self.max_steer = np.radians(60)
        self.max_d_steer = np.radians(60)
