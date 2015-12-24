# Eric Watson - Follow Car Fun Project
# Project Description is here: https://docs.google.com/document/d/1NkPps1JuIdaNQKTh2MQQV7bjhTHmIg6AfeSybolur6A/edit?usp=sharing
# Stage 1 Simulation: Get car A to the navigation goal - find brightest light

import numpy as np
import math
import matplotlib.pyplot as plt

# define functions needed
def unit (vector):
    """
    Args:
        vector: array you want the unit vector of
    
    Returns: 
        unit: vector in direction of input vector but with magnitude of 1
    """
    mag = math.sqrt(np.dot(vector, vector))
    if mag == 0:
        return 0
    return vector / mag

# initialize simulation
TIME_STEP = 0.1				
TIME_FINAL = 10
time = np.arange(0., TIME_FINAL + TIME_STEP, TIME_STEP)
N_STEPS = len(time)
a_state_inertial = np.zeros((N_STEPS, 4))    # initialized state vector [posX; posY; velX; velY]
CONSTANT_VELOCITY = 0.5                         # m/s
a_state_inertial[0] = [0, 0, 0, CONSTANT_VELOCITY]		# car A starts at the origin moving at constant speed +Y
bright_light = np.array([4., -1.])			# bright light fixed position in inertial - navigation goal is to get there!	

# sensor reading setup - only knows if left or right is brighter
sensor_reading = np.zeros(N_STEPS)	# +1 for left turn, -1 for right turn

# vehicle dynamics
# assume car is at a constant velocity, and max turning control is an angular rate of change of 20 degrees per second
MAX_HEADING_CHANGE_PER_SECOND = math.radians(20) 	# radians / sec max change in body coordinates
MAX_HEADING_CHANGE_PER_TIME_STEP = MAX_HEADING_CHANGE_PER_SECOND * TIME_STEP
MAX_CROSS_TRACK_VELOCITY = CONSTANT_VELOCITY * math.sin(MAX_HEADING_CHANGE_PER_TIME_STEP)	
MAX_ALONG_TRACK_VELOCITY = math.sqrt(CONSTANT_VELOCITY ** 2 - MAX_CROSS_TRACK_VELOCITY ** 2)

# run simulation determining sensor reading, applying control law, changing
# velocity based on control, and integrating velocity to determine new position
for i, t in enumerate(time): 	
    if i > 0:     # starting at second time step...
        # Determine if brighter light on left or right of body along track direction
        a_to_bright_light = bright_light - a_state_inertial[i-1, 0:2]	# vector from car A to bright light in inertial
        sign_of_heading_angle_to_bright_light = a_state_inertial[i-1, 2] * a_to_bright_light[1] - a_state_inertial[i-1, 3] * a_to_bright_light[0]   # "cross product" to determine sign of angle between heading and brihgt light in body
        if sign_of_heading_angle_to_bright_light >= 0:
            sensor_reading[i] = 1   # bright light is left
        else:
            sensor_reading[i] = -1  # bright light is right
        
        # implement bang bang control via cross track velocity (proportional to wheel turning motor voltage)
        a_velocity_old_unit = unit(a_state_inertial[i-1, 2:4])	# inertial velocity unit vector from previous time step
        if i == 1:  # still going max speed, haven't turned yet
            a_velocity_old = CONSTANT_VELOCITY * a_velocity_old_unit
        else:
            a_velocity_old = MAX_ALONG_TRACK_VELOCITY * a_velocity_old_unit			# “slow down to turn”
        a_state_inertial[i, 2:4] = a_velocity_old + (sensor_reading[i] * np.array([-a_velocity_old_unit[1], a_velocity_old_unit[0]]) * MAX_CROSS_TRACK_VELOCITY)
        
        # integrate to determine inertial position
        a_state_inertial[i, 0:2] = a_state_inertial[i - 1, 0:2] + a_state_inertial[i, 2:4] * TIME_STEP

# plot simulation outputs

plt.figure(figsize=(8, 6), dpi=100) # plot state in time vs. X, Y, vX, vY

plt.subplot(2, 1, 1)
plt.plot(time, a_state_inertial[:, 0], label="$X$")
plt.plot(time, a_state_inertial[:, 1], label="$Y$")
plt.ylabel('position [m]')
plt.legend(frameon=False, loc='top right')

plt.subplot(2, 1, 2)
# get velocity magnitude
a_velocity_mag = [math.sqrt(state[2] ** 2 + state[3] ** 2) for state in a_state_inertial]
plt.plot(time, a_state_inertial[:, 2], label="$v_X$")
plt.plot(time, a_state_inertial[:, 3], label="$v_Y$")
plt.plot(time, a_velocity_mag, 'k', label="$v_{mag}$")
plt.xlabel('time [s]')
plt.ylabel('velocity [m/s]')
plt.legend(frameon=False, loc='bottom left')

plt.show()

plt.figure(figsize=(9, 6), dpi=100)   # plot position in X vs Y

plt.plot(a_state_inertial[:, 0], a_state_inertial[:, 1], 'k')
plt.plot(a_state_inertial[0, 0], a_state_inertial[0, 1], 'ko')
plt.plot(a_state_inertial[N_STEPS - 1, 0], a_state_inertial[N_STEPS - 1, 1], 'kx')
plt.plot(bright_light[0], bright_light[1], 'ro')
plt.xlabel('X [m]')
plt.ylabel('Y [m]')

plt.axis('equal')

plt.show()

plt.figure(figsize=(8, 6), dpi=100)   # plot control 

plt.plot(time, sensor_reading, 'ko')
plt.ylim((-1.1, 1.1))
plt.xlim((-0.1, 10))
plt.axhline(1, label="left", color='c')
plt.axhline(-1, label="right", color='r')
plt.xlabel('time [s]')
plt.ylabel('Turn Control')
plt.legend(frameon=False, loc='center right')
plt.show()
