# -*- coding: utf-8 -*-
"""
Created on Sun Nov  8 07:53:23 2015

@author: ericwatson
"""

# Eric Watson's Following Car Project

# import modules
import pyb

# setup USR interupt switch to toggle LED #2

switch = pyb.Switch()

def light2():
    pyb.LED(2).toggle()
switch.callback(light2)

# TODO - setup timer to run at ~5HZ and measure photo diode on that timer


# setup ADC on pin 12 
pin_photo1_X12 = pyb.Pin.board.X12
adc_photo1 = pyb.ADC(pin_photo1_X12)

# read + print value from photo 1
val_photo1 = adc_photo1.read()
print(val_photo1)

# print if it's bright or not
if val_photo1 > 1500:
    print("It is getting hot in here!")
else:
    print("Brrrrrr, it is cold as in here")






