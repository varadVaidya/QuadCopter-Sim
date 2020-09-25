import numpy as np
from QuadCopter import QuadRotor

class controlDynamics:
    
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
        
            
        
        The current plan is to use the dynamics function to solve for all the variables and then store then in the current_state
        class for future use.
        
        '''
        
    def __init__(self):
            super().__init__()
    
    def attitude_solver(self,attitude,u):
        
        '''
        attitude should contain 
        roll pitch yaw, rolldot,pitchdot,yawdot
        '''
        
        roll,pitch,yaw,rolldot,pitchdot,yawdot = attitude # Unpack the attitude vector
        
        u1,u2,u3,u4 = u      # unpack the input vector
        
        attitude_derivative = np.array([rolldot,pitchdot,yawdot,
                                u2/QuadRotor().I[0,0], u3/QuadRotor().I[1,1], u4/QuadRotor().I[2,2] ])
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
        
        position_derivative = np.array([xdot,ydot,zdot,
                            (u1/QuadRotor().mass) * ( np.cos(phi) * np.sin(theta) * np.cos(psi) + np.sin(phi) * np.sin(psi) ) ,
                            (u1/QuadRotor().mass) * ( np.cos(phi) * np.sin(theta) * np.sin(psi) - np.sin(phi) * np.cos(psi) )
                            (u1/QuadRotor().mass) * ( np.cos(phi) * np.cos(theta) - QuadRotor().gravity )
                            ])
        
        return position_derivative

    