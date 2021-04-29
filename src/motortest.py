import RPi.GPIO as GPIO
import numpy as np
import matplotlib.pyplot as plt
import time

if True:
    GPIO.setmode(GPIO.BCM) ## this is to keep the pinnums consistent with the encoder reading
    dirt=22
    step=27
    GPIO.setup(dirt, GPIO.OUT)
    GPIO.setup(step, GPIO.OUT)

    motorinc=1.8*2*np.pi/180 #in radians, i think this is 1.8 deg
    shaftradius=0.0025

    shutter=0.1

    GPIO.output(dirt,GPIO.HIGH)
    for i in range(200):
        time.sleep(shutter)
        GPIO.output(step,GPIO.HIGH)
        time.sleep(shutter)
        GPIO.output(step,GPIO.LOW)
    GPIO.output(dirt,GPIO.LOW)
    for i in range(200):
        time.sleep(shutter)
        GPIO.output(step,GPIO.HIGH)
        time.sleep(shutter)
        GPIO.output(step,GPIO.LOW)
    GPIO.output(step,GPIO.LOW)

## put like, a tape flag on the motor. what you should expect is it spins 180 degrees in one direction
## Then turns around and spins 180 decrees back to the original position

