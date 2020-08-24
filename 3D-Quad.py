'''
Goal of this project is to develop the dynamics of a quadcopter and control it.
And add trajectory tracking
'''

import numpy as np
from scipy.integrate import odeint

class QuadRotor:
    
    def __init__(self):
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
        self.invI = np.linalg.inv(self.I)
        self.arm_length = 0.086 #m        
        # All the parameters for the drone are taken from the Aerial Robotics Course
        self.init_state = self.state()
        self.curr_state = self.state()
        
        # create a state class
    class state:
        def __init__(self):
            self.pos = np.array([0,0,0])
            self.vel = np.array([0,0,0])
            self.accel = np.array([0,0,0])
            self.rot = np.array([0,0,0])
            self.omega = np.array([0,0,0])
            self.yaw = 0
            self.yawdot = 0
            #set the class and name all variable required according to the dynamics
                
    
    def dynamics(self,state,u):
        state_derivative = 0
        return state_derivative

        