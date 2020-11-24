'''
Simulates the model created in model.Quadcopter using the dynamics provided in the model.dynamics

'''
import numpy as np
from model.QuadCopter import QuadRotor
from model.dynamics import controlDynamics
# import model.QuadCopter
# import model.dynamics
from scipy.integrate import odeint

### SIMULATION TIME PARAMETERS ####
total_time = 10
delta_t = 0.01
timepoints = int(total_time/delta_t + 1)

t = np.linspace(0,total_time,timepoints)

####


### FIND THE INPUT VECTOR

u = np.array([0,0,0,0])




# setting all stuff to zero to check if all stuff works
 
###
#Drone = model.QuadCopter.QuadRotor()

Drone = QuadRotor()

for i in range(len(t)):
    
    #curr_state = Drone.curr_state
    curr_attitude = np.array([Drone.curr_state.attitude,Drone.curr_state.pqr]).reshape(6,)
    curr_position = np.array([Drone.curr_state.pos,Drone.curr_state.vel]).reshape(6,)
    
    
    attitude = odeint(controlDynamics.attitude_solver  ,curr_attitude , t, args=(u,))
    position = odeint(controlDynamics.position_solver  ,curr_position , t, args=(u,Drone.curr_state.attitude), )
    
    
    






