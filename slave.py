# main.py -- put your code here!

import pyb

# initialize LED
green = pyb.LED(2)          # create green LED object
blue = pyb.LED(4)

# Initialize UART
uart = pyb.UART(1, 9600)                        # init with given baudrate
uart.init(9600, bits=8, parity=None, stop=1)    # init with given parameters
data = bytearray(1)                             # init data variable to store

blue.on()

# read and turn on LED
uart.readinto(data)
if data == b'1':
    green.on()

