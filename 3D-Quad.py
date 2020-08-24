'''
Goal of this project is to develop the dynamics of a quadcopter and control it.
And add trajectory tracking
'''

import numpy as np
from scipy.integrate import odeint

class QuadRotor:
    
    def __init__(self):
        super().__init__()
        self.mass = 0.1800
        self.gravity = 9.81
        self.Ixx = 0.00025
        self.maxF = 3.5316
        self.minF = 0.0
        self.I = np.array([
            [0.00025, 0       ,0],
            [0      , 0.000232,0],
            [0, 0       ,0.0003738]
        ])
        self.invI = np.linalg.inv(I)
        self.arm_length = 0.086 #m
        # All the parameters for the drone are taken from the Aerial Robotics Course
        
    # create an inital state class
    class init_state:
        def __init__(self):
            pos = np.array([0,0,0])
            vel = np.array([0,0,0])
            accel = np.array([0,0,0])
            rot = np.array([0,0,0])
            omega = np.array([0,0,0])
            yaw = 0 
            super().__init__()
        
        
        