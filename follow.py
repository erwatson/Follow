# follow.py
# Eventually will have one car follow another wirelessly using shared sensors

# Eric Watson's Following Car Project

# import modules
import pyb

# creating objects
green = pyb.LED(2)          # create green LED object
switch = pyb.Switch()       # create switch object

# Initialize UART
uart = pyb.UART(1, 9600)                        # init with given baudrate
uart.init(9600, bits=8, parity=None, stop=1)    # init with given parameters

# setup ADC on pin 12 
pin_photo1_X12 = pyb.Pin.board.X12
adc_photo1 = pyb.ADC(pin_photo1_X12)

# initialize data collection 
green.on()                          # shows recording data
log = open('/sd/log.csv', 'w+')     # open file on SD 
log.write('{},{}\n'.format("t","L1"))   # write column descriptions to file 

# setup buffer to send for flashing light on slave
# buffer = b'1'      # light starts on

uart.write(b'1')


# run until switch is pressed again
while not switch():
    t = pyb.millis()                            # get time
    val_photo1 = adc_photo1.read()              # read value from photo sensor 1
    log.write('{},{}\n'.format(t,val_photo1))   # write data to file       
    pyb.delay(100)                              # sample about 10 Hz
#    if buffer == b'1':                          # make buffer toggle high and low
#        buffer = b'0'
#        green.on()
#    elif buffer == b'0':
#        buffer = b'1'
#        green.off()
#    uart.write(buffer)
    
# end after switch press
log.close()                         # close file
green.off()                         # green LED off shows file closed
pyb.delay(200)                      # delay avoids detection of multiple presses