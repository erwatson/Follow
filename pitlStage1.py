# Eric Watson - Follow Car Fun Project
# Project Description is here: https://docs.google.com/document/d/1NkPps1JuIdaNQKTh2MQQV7bjhTHmIg6AfeSybolur6A/edit?usp=sharing
# Stage 1: Get car A to the navigation goal - find brightest light with 
# bang-bang control based on brighter of two light sensors
        
# The only things that need to be run on the pyboard are:
    # 1) find brighter side (left or right)
    # 2) control car to turn in this direction
    # 3) save control direction commanded and time stamp
    
    # Everything else will take care of itself
    # I will first create random sensor output readings, determine which side
    # is brighter, control car in this direction, and save these values  
        
# Next hardware steps are:
    # 1) getting actual sensor readings to work and showing it with a 
    # flashlight - I can shine a light while it's recording a turn direction 
    # and then check that it's responding correctly
    # 2) get the boards to talk to each other and have A pass it's control 
    # info along to B, then have B write both with appropriate time stamps...
        
        
# import modules       
import pyb

# creating objects
green = pyb.LED(2)          # create green LED object
switch = pyb.Switch()       # create switch object

# initialize data collection 
green.on()                          # shows recording data
log = open('/sd/control_log.csv', 'w+')     # open file on SD 
log.write('{},{},{},{}\n'.format("t", "left_sensor", "right_sensor", "control"))   # write column descriptions to file 

# run until switch is pressed again
while not switch():
    t = pyb.millis()       
    # set random sensor readings based on time   
    t_string = str(t)                   
    t_randomized = str(t * (int(t_string[1]) + 1))
    t_len = len(t_randomized)
    val_photo_left = int(t_randomized[t_len - 1])
    val_photo_right = int(t_randomized[t_len - 2])
    if val_photo_left >= val_photo_right:
        control = int(1)
    else:
        control = int(-1)
    log.write('{},{},{},{}\n'.format(t, val_photo_left, val_photo_right, control))   # write data to file       
    pyb.delay(100)                              # sample about 10 Hz
    
# end after switch press
log.close()                         # close file
green.off()                         # green LED off shows file closed
pyb.delay(200)                      # delay avoids detection of multiple presses
        


