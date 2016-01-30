# Eric Watson - Follow Car Fun Project
# Project Description is here: https://docs.google.com/document/d/1NkPps1JuIdaNQKTh2MQQV7bjhTHmIg6AfeSybolur6A/edit?usp=sharing

# CAR B PITL STAGE 2
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
orange = pyb.LED(3)         # error light
switch = pyb.Switch()       # create switch object
FOLLOW_TIME = 1000          # milliseconds
UART_READ_TIME = 100
DELAY_TIME = 0              # milliseconds
delay_steps = int(round(FOLLOW_TIME / (DELAY_TIME + UART_READ_TIME)))
uart2 = pyb.UART(2, 9600)
uart2.init(9600, bits=8, parity=None, stop=1, timeout=UART_READ_TIME)
controlA = []

# initialize data collection 
green.on()                          # shows recording data
log = open('/sd/control_log.csv', 'w+')     # open file on SD 
log.write('{},{},{}\n'.format("t", "controlA", "controlB"))   # write column descriptions to file 

t_init = pyb.millis()
count = 0
# run until switch is pressed again
while not switch():
    green.toggle()
    control_temp = uart2.read()
    if len(control_temp) == 0:
        orange.on()
        controlA.append(int(2))
    else:
        if control_temp[0] == 48:
            controlA.append(int(-1))
            orange.off()
        elif control_temp[0] == 49:
            controlA.append(int(1))
            orange.off()
        else:
            orange.on()
            controlA.append(int(3))
             
    if count < delay_steps:
        controlB = 0
    else:
        controlB = controlA[len(controlA) - delay_steps - 1] 
    t = pyb.millis()  
    log.write('{},{},{}\n'.format(t, controlA[len(controlA) - 1], controlB))
    pyb.delay(DELAY_TIME)                              # sample about 10 Hz with UART_READ_TIME
    count += 1
# end after switch press
log.close()                         # close file
green.off()                         # green LED off shows file closed
pyb.delay(200)                      # delay avoids detection of multiple presses
        


