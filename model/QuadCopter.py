'''
Goal of this project is to develop the dynamics of a quadcopter and control it.
And add trajectory tracking
'''

import numpy as np


class QuadRotor:
    
    def __init__(self):
        self.mass = 0.1800
        self.gravity = 9.81
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
        
        #init the class of current state and current state.
        self.init_state = self.state()
        self.curr_state = self.state()
        
        # create a state class
    class state:
        def __init__(self):
            # POSITION VELOCITY AND ACCEL IN THE FORMAT OF X,Y,Z
            self.pos = np.array([0,0,0])
            self.vel = np.array([0,0,0])
            self.accel = np.array([0,0,0])
            #  ATTITUDE IN THE FORMAT OF ROLL PITCH YAW.
            self.attitude = np.array([0,0,0])
            # PQR is the angluar velocity in the body frame.
            self.pqr = np.array([0,0,0])
            

            #set the class and name all variable required according to the dynamics
                
    
    def dynamics(self,state,u):
        '''
        the control dynamics of 3D quadrotor can be be written as:
        xddot     [ 0 ]         [ 0 ] 
        yddot  =  [ 0 ]  +  R * [ 0 ] 
        zddot     [-g ]         [u_1] 
        
        where, R is the rotation matrix that maps stuff from body frame to world frame.
        
        [p]      [c(theta)  0  -c(phi)s(theta)]     [ phidot   ]
        [q]    = [    0     1    sin(phi)     ]  *  [ thetadot ]
        [r]      [s(theta)  0  c(phi)c(theta) ]     [ psidot   ]
        
        where p,q,r is the body frame angular velocities.
        
            [pdot]     [u_2]    [p]     [p]
        I * [qdot]  =  [u_3]  - [q] * I [q]
            [rdot]     [u_4]    [r]     [r]
            
        u1,u2,u3,u4 are the control input to the system. 
        
        
        assuming the hover position at all times
        that means the theta and phi are at all times zero.
        
        thus pqr will be equal to the derivative of the roll pitch yaw.
        
            
        '''
        
        '''
        The current plan is to use the dynamics function to solve for all the variables and then store then in the current_state
        class for future use.
        
        '''
        def attitude_solver(self,attitude,u):
            '''
            attitude should contain 
            roll pitch yaw, rolldot,pitchdot,yawdot
            '''
            
            roll,pitch,yaw,rolldot,pitchdot,yawdot = attitude # Unpack the attitude vector
            
            u1,u2,u3,u4 = u      # unpack the input vector
            
            attitude_derivative = [rolldot,pitchdot,yawdot,
                                   u2/self.I[0,0], u3/self.I[1,1], u4/self.I[2,2] ]
            return attitude_derivative
        
        def position_solver(self,positon,u,attitude):
            '''
            position must contain
            x , y, z , xdot,ydot,zdot
            attitude must contain
            phi,theta,psi
            '''
            
            x,y,z,xdot,ydot,zdot = positon
            u1,u2,u3,u4 = u      # unpack the input vector
            phi,theta,psi = attitude
            
            position_derivative =[xdot,ydot,zdot,
                                  (u1/self.mass) * ( np.cos(phi) * np.sin(theta) * np.cos(psi) + np.sin(phi) * np.sin(psi) ) ,
                                  (u1/self.mass) * ( np.cos(phi) * np.sin(theta) * np.sin(psi) - np.sin(phi) * np.cos(psi) )
                                  (u1/self.mass) * ( np.cos(phi) * np.cos(theta) - self.gravity )
                                  ]
            
            return position_derivative

        