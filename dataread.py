# dataread.py
# Does nothing, but turn on the red light showing we've entered data read mode
import pyb

# turn on red light
red = pyb.LED(1)        # create red LED object
red.on()                # turn on the red light!