# -*- coding: utf-8 -*-
"""
Created on Sun Nov  8 07:53:23 2015

@author: ericwatson
"""

# Eric Watson's Following Car Project

# import modules
import pyb

# initialize Constants
THRESH = 6

# setup USR interupt switch to toggle LED #2

switch = pyb.Switch()

def light2():
    pyb.LED(2).toggle()
switch.callback(light2)

# get acceleration and make level

accel = pyb.Accel()
lightX = pyb.LED(1)
lightY = pyb.LED(4)

while True:
    if abs(accel.x()) > THRESH:
        lightX.on()
    else:
        lightX.off()
    if abs(accel.y()) > THRESH:
        lightY.on()
    else:
        lightY.off()
    
    pyb.delay(100)

