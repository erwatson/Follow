# boot.py -- runs on boot-up
# Select which scripts to run, with some extra logic
# > To run 'follow.py':
#       * press reset and do nothing else
# > To run 'dataread.py':
#       * press reset
#       * press user switch and hold until orange LED goes out

import pyb

pyb.LED(3).on()                 # indicate we are waiting for switch press
pyb.delay(2000)                 # wait for user to maybe press the switch
switch = pyb.Switch()()   # sample the switch at end of delay
pyb.LED(3).off()                # indicate that we finished waiting for the switch

pyb.LED(4).on()                 # indicate that we are selecting the mode

if switch:
    pyb.usb_mode('CDC+MSC')
    pyb.main('dataread.py')           # if switch was pressed, run this
else:
    pyb.usb_mode('CDC+HID')
    pyb.main('follow.py')           # if switch wasn't pressed, run this

pyb.LED(4).off()                # indicate that we finished selecting the mode
