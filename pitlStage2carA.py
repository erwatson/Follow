# Eric Watson - Follow Car Fun Project
# Project Description is here: https://docs.google.com/document/d/1NkPps1JuIdaNQKTh2MQQV7bjhTHmIg6AfeSybolur6A/edit?usp=sharing

# CAR A PITL STAGE 2
# Stage 2: Get car A to the navigation goal - find brightest light with 
# bang-bang control based on brighter of two light sensors AND have car B 
# copy the control of car A with a time delay to follow car A to the goal
        
# The only things that need to be run on the pyboards are:
    # 1) car A finds brighter side (left or right)
    # 2) car A controls to turn in this direction
    # 3) car A transmit control direction to car B
    # 4) car B determines its control based on follow time and car A control
    # 5) car B saves each cars control decision and the time stamp at each step
    
    # Everything else will take care of itself
    # I will first create random sensor output readings, determine which side
    # is brighter, control car in this direction, and save these values  
        
# Next hardware step is:
    # 1) get actual sensor readings to work and showing it with a 
    # flashlight - I can shine a light while it's recording a turn direction 
    # and then check that it's responding correctly
        
        
# import modules       
import pyb

# creating objects
green = pyb.LED(2)          # create green LED object
switch = pyb.Switch()       # create switch object
DELAY_TIME = 0            # milliseconds
UART_READ_TIME = 100
uart4 = pyb.UART(4, 9600)
uart4.init(9600, bits=8, parity=None, stop=1, timeout=UART_READ_TIME)

green.on()                          # shows recording data

# run until switch is pressed again
while not switch():
    green.toggle()
    t = pyb.millis()       
    # set random sensor readings based on time   
    t_string = str(t)                   
    t_randomized = str(t * (int(t_string[1]) + 1))
    t_len = len(t_randomized)
    val_photo_left = int(t_randomized[t_len - 1])
    val_photo_right = int(t_randomized[t_len - 2])
    if val_photo_left >= val_photo_right:
        control = b'1'
    else:
        control = b'0'
        
    # write to UART
    uart4.write(control) 
    pyb.delay(DELAY_TIME)                              # sample about 10 Hz
    
# end after switch press
green.off()                         # green LED off shows file closed
pyb.delay(200)                      # delay avoids detection of multiple presses
        


