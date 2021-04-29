#!/usr/bin/env python3
# movemotor.py - moves motor pending on user input
# 05/23/21 Richard Yang

import sys
import RPi.GPIO as GPIO
import numpy as np
import time

GPIO.setmode(GPIO.BCM) ## this is to keep the pinnums consistent with the encoder reading
dirt=22
step=27
GPIO.setup(dirt,GPIO.OUT)
GPIO.setup(step,GPIO.OUT)
motorinc=1.8*2*np.pi/180 #in radians, I think this is 1.8 deg
shaftradius=0.0025
shutter=0.0005 #min=0.0005

move = input('Enter distance [mm]: ')
if move:
    try:
        move = float(move)
        move = 5*move
        move = int(move)
    except ValueError:
        print('Inpur format invalid.\n')
    if move<0: #ccw (away from motor)
        move = abs(move)
        GPIO.output(dirt,GPIO.LOW)
        for i in range(move):
            time.sleep(shutter)
            GPIO.output(step,GPIO.HIGH)
            time.sleep(shutter)
            GPIO.output(step,GPIO.LOW)
    else:
        GPIO.output(dirt,GPIO.HIGH)
        for i in range(move):
            time.sleep(shutter)
            GPIO.output(step,GPIO.HIGH)
            time.sleep(shutter)
            GPIO.output(step,GPIO.LOW)
else:
    print('Exited\n')
