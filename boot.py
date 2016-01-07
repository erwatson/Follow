# boot.py -- runs on boot-up
# Select which scripts to run, with some extra logic
# > To run 'follow.py':
#       * press reset and do nothing else
# > To run 'dataread.py':
#       * press reset
#       * press user switch and hold until orange LED goes out

import pyb

orange = pyb.LED(3)         # create orange LED object
orange.on()                 # indicate we are waiting for switch press
pyb.delay(2000)             # wait for user to maybe press the switch
switch = pyb.Switch()       # create switch object
switch_val = switch()       # sample the switch at end of delay
orange.off()                # indicate that we finished waiting for the switch
blue = pyb.LED(4)           # create blue LED object
blue.on()                   # indicate that we are selecting the mode

if switch_val:
    pyb.usb_mode('CDC+MSC')
    pyb.main('dataread.py')           # if switch was pressed, run this
else:
    pyb.usb_mode('CDC+HID')
    pyb.main('pitlStage1.py')           # if switch wasn't pressed, run this

blue.off()                # indicate that we finished selecting the mode
