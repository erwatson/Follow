# UART Loopback test

import pyb

# initialize UARTs
uart4 = pyb.UART(4, 9600)       # init with given properties
uart2 = pyb.UART(2, 9600)       # init with given properties

uart4.init(9600, bits=8, parity=None, stop=1) 
uart2.init(9600, bits=8, parity=None, stop=1)

uart4.write(b'123456')
uart2.read()