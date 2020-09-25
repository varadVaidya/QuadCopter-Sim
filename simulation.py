'''
Simulates the model created in model.Quadcopter using the dynamics provided in the model.dynamics

'''
import numpy as np
import model.QuadCopter
import model.dynamics

### SIMULATION TIME PARAMETERS ####
total_time = 10
delta_t = 0.01
timepoints = total_time/delta_t
####


### FIND THE INPUT VECTOR

u = np.array([0,0,0,0])
# setting all stuff to zero to check if all stuff works
 
###
Drone = model.QuadCopter.QuadRotor()




