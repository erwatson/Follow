# follow.py
# Eventually will have one car follow another wirelessly using shared sensors

# Eric Watson's Following Car Project

# import modules
import pyb

# creating objects
green = pyb.LED(2)
switch = pyb.Switch()
timer = pyb.Timer(4)

# setup timer
tim.init(freq=10)                   # sets timer freq to 10 Hz

# setup ADC on pin 12 
pin_photo1_X12 = pyb.Pin.board.X12
adc_photo1 = pyb.ADC(pin_photo1_X12)

# initialize data collection 
green.on()                          # shows recording data
log = open('/sd/log.csv', 'w+')     # open file on SD 

# create function to get and record data
def sampleRecord():
    t = pyb.millis()                            # get time
    val_photo1 = adc_photo1.read()              # read value from photo sensor 1
    log.write('{},{}\n'.format(t,val_photo1))   # write data to file    

# start data collection
timer.callback(sampleRecord)        # runs sampleRecord funtion on timer event

# run until switch is pressed again
while not switch():
    pyb.delay(50)                   # do nothing until switch is pressed again
    
# stop timer callback
timer.callback(None)
    
# end after switch press
log.close()                         # close file
green.off()                          # blue LED off shows file closed
pyb.delay(200)                      # delay avoids detection of multiple presses