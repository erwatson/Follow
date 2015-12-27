# Eric Watson - Follow Car Fun Project
# Project Description is here: https://docs.google.com/document/d/1NkPps1JuIdaNQKTh2MQQV7bjhTHmIg6AfeSybolur6A/edit?usp=sharing
# Stage 1 Simulation: Get car A to the navigation goal - find brightest light
# Stage 2 Simulation: Get car B to follow car A to navigation objective

import numpy as np
import math
import matplotlib.pyplot as plt

# define functions needed
def unit(vector):
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
    
class Vehicle(object):
    """A vehicle with certain sensors and different levels of autonomy with 
    the following properties:
    
    Attributes:
        name: a string representing the vehicles name
        CONSTANT_VELOCITY: a float representing the constant velocity magnitude
            of the vehicle
        MAX_HEADING_CHANGE_PER_SECOND: a float representing the maximum angle 
            change per second of the vehicle in radians
    """
    
    def __init__(self, name, CONSTANT_VELOCITY, MAX_HEADING_CHANGE, 
                 INITIAL_STATE, N_STEPS, TIME_STEP):
        """Return a vehicle object that's name is string *name*, has constant
        velocity float *CONSTANT_VELOCITY* in m/s, max heading change float 
        *MAX_HEADING_CHANGE* in rad/s, max heading change per *TIME_STEP* of 
        float MAX_HEADING_CHANGE_PER_STEP in radians per step, cross track
        velocity when under control per step of *CONTROL_CROSS_TRACK_VELOCITY*,
        along track velocity when under control of float 
        *CONTROL_ALONG_TRACK_VELOCITY*, has an initialized array state with
        dimensions 4 x *N_STEPS* with vector [posX, posY, velX, velY] and first 
        state initialized to INITIAL_STATE, and sensor reading output array 
        *sensor_reading* with length *N_STEPS*
        """
        
        self.name = name
        self.CONSTANT_VELOCITY = CONSTANT_VELOCITY
        self.MAX_HEADING_CHANGE = MAX_HEADING_CHANGE  # radians/sec
        self.MAX_HEADING_CHANGE_PER_STEP = MAX_HEADING_CHANGE * TIME_STEP
        self.CONTROL_CROSS_TRACK_VELOCITY = CONSTANT_VELOCITY * math.sin(self.MAX_HEADING_CHANGE_PER_STEP)
        self.CONTROL_ALONG_TRACK_VELOCITY = math.sqrt(CONSTANT_VELOCITY ** 2 - self.CONTROL_CROSS_TRACK_VELOCITY ** 2)
        self.state = np.zeros([N_STEPS, 4])     # initialized state vector [posX; posY; velX; velY]
        self.state[0] = INITIAL_STATE
        # sensor reading setup - only knows if left or right is brighter 
        self.sensor_reading = np.zeros(N_STEPS) # +1 for left turn, -1 for right turn
        
    def control_direction(self, bright_light, step):
        """Determines if brighter light is to the left or right and fills in the
        sensor_reading array"""
        a_to_bright_light = bright_light - self.state[step-1, 0:2]	# vector from car A to bright light in inertial
        sign_of_heading_angle_to_bright_light = self.state[step-1, 2] * a_to_bright_light[1] - self.state[step-1, 3] * a_to_bright_light[0]   # "cross product" to determine sign of angle between heading and brihgt light in body
        if sign_of_heading_angle_to_bright_light >= 0:
            self.sensor_reading[step] = 1   # bright light is left
        else:
            self.sensor_reading[step] = -1  # bright light is right
            
    def new_state(self, step, TIME_STEP):
        """Determine next state in time step via next control velocity and 
        integrating for position"""
        # implement bang bang control via cross track velocity (proportional to wheel turning motor voltage)
        a_velocity_old_unit = unit(self.state[step-1, 2:4])	# inertial velocity unit vector from previous time step
        if step == 1:  # still going max speed, haven't turned yet
            a_velocity_old = self.CONSTANT_VELOCITY * a_velocity_old_unit
        else:
            a_velocity_old = self.CONTROL_ALONG_TRACK_VELOCITY * a_velocity_old_unit			# “slow down to turn”
        self.state[step, 2:4] = a_velocity_old + (self.sensor_reading[step] * np.array([-a_velocity_old_unit[1], a_velocity_old_unit[0]]) * self.CONTROL_CROSS_TRACK_VELOCITY)
        
        # integrate to determine inertial position
        self.state[step, 0:2] = self.state[step - 1, 0:2] + self.state[i, 2:4] * TIME_STEP
        
    def velocity_mag(self):
        """Calculates the velocity magnitude of the state and returns it"""
        return [math.sqrt(state[2] ** 2 + state[3] ** 2) for state in self.state]
    

# initialize simulation
TIME_STEP = 0.1				
TIME_FINAL = 10
time = np.arange(0., TIME_FINAL + TIME_STEP, TIME_STEP)
N_STEPS = len(time)   

# vehicle dynamics
# assume car is at a constant velocity, and max turning control is an angular rate of change of 20 degrees per second
CONSTANT_VELOCITY = 0.5                         # m/s
MAX_HEADING_CHANGE = math.radians(20) 	# radians / sec max change in body coordinates

# initialize car A object
A_INITIAL_STATE = [0, 0, 0, CONSTANT_VELOCITY]		# car A starts at the origin moving at constant speed +Y
a = Vehicle("car_a", CONSTANT_VELOCITY, MAX_HEADING_CHANGE, A_INITIAL_STATE, N_STEPS, TIME_STEP) 

BRIGHT_LIGHT = np.array([4., -1.])			# bright light fixed position in inertial - navigation goal is to get there!	

# run simulation determining sensor reading, applying control law, changing
# velocity based on control, and integrating velocity to determine new position
for i, t in enumerate(time): 	
    if i > 0:     # starting at second time step...
        # Determine if brighter light on left or right of body along track direction
        a.control_direction(BRIGHT_LIGHT, i)
        
        # implement bang bang control via cross track velocity (proportional to wheel turning motor voltage)
        a.new_state(i, TIME_STEP)
        
# plot simulation outputs

# plot state in time vs. X, Y, vX, vY
plt.figure(figsize=(8, 6), dpi=100) 

plt.subplot(2, 1, 1)
plt.plot(time, a.state[:, 0], label="$X$")
plt.plot(time, a.state[:, 1], label="$Y$")
plt.ylabel('position [m]')
plt.legend(frameon=False, loc='top right')

plt.subplot(2, 1, 2)
# get velocity magnitude
a_velocity_mag = a.velocity_mag()
plt.plot(time, a.state[:, 2], label="$v_X$")
plt.plot(time, a.state[:, 3], label="$v_Y$")
plt.plot(time, a_velocity_mag, 'k', label="$v_{mag}$")
plt.xlabel('time [s]')
plt.ylabel('velocity [m/s]')
plt.legend(frameon=False, loc='bottom left')

plt.show()

# plot position in X vs Y
plt.figure(figsize=(9, 6), dpi=100)   

plt.plot(a.state[:, 0], a.state[:, 1], 'k')
plt.plot(a.state[0, 0], a.state[0, 1], 'ko')
plt.plot(a.state[N_STEPS - 1, 0], a.state[N_STEPS - 1, 1], 'kx')
plt.plot(BRIGHT_LIGHT[0], BRIGHT_LIGHT[1], 'ro')
plt.xlabel('X [m]')
plt.ylabel('Y [m]')

plt.axis('equal')

plt.show()

# plot control 
plt.figure(figsize=(8, 6), dpi=100)   

plt.plot(time, a.sensor_reading, 'ko')
plt.ylim((-1.1, 1.1))
plt.xlim((-0.1, 10))
plt.axhline(1, label="left", color='c')
plt.axhline(-1, label="right", color='r')
plt.xlabel('time [s]')
plt.ylabel('Turn Control')
plt.legend(frameon=False, loc='center right')
plt.show()
