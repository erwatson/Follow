# follow.py
# Eventually will have one car follow another wirelessly using shared sensors

# Eric Watson's Following Car Project

# import modules
import pyb

# creating objects
green = pyb.LED(2)
switch = pyb.Switch()

# setup ADC on pin 12 
pin_photo1_X12 = pyb.Pin.board.X12
adc_photo1 = pyb.ADC(pin_photo1_X12)

# start data collection 
green.on()                          # shows recording data
log = open('/sd/log.csv', 'w+')     # open file on SD 

# run until switch is pressed again
while not switch():
    t = pyb.millis()                            # get time
    val_photo1 = adc_photo1.read()              # read value from photo sensor 1
    log.write('{},{}\n'.format(t,val_photo1))   # write data to file
    
# end once switch is pressed again
log.close()                         # close file
green.off()                          # blue LED off shows file closed
pyb.delay(200)                      # delay avoids detection of multiple presses