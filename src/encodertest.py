## Hello Richard, this is a little code for you to test out
## But there are some IRL instructions too!!

## 1. Wedge one of the shafts into the encoder. Make sure you have a grip and can turn it and stuff. 
## Lock it in there

## 2. Using the female to female connector cables (you should have those from the lab class or the order)
## Connect ground and 5V from the RPi to the optical encoder. http://www.farnell.com/datasheets/1884440.pdf
## is an example datasheet. Use GND and Vcc pins for this

## 3. Connect the CH A and CH B pins from the encoder into GPIO Pins 23 and 24 into the RPi.
## which one to which pin is not important yet
## Here https://www.raspberrypi.org/documentation/usage/gpio/
## is a datasheet for that

## 4. Using pip or conda or your preferred installation method, install Encoder. In pip this looks like
## $ pip install Encoder
## Looking to specifically use this package https://pypi.org/project/Encoder/

## 5. Run this code

#!/usr/bin/env python3
import sys
import Encoder 
import RPi.GPIO as GPIO
import numpy as np
import matplotlib.pyplot as plt
import time

enc = Encoder.Encoder(24,23)
while True:
    print(enc.read())


#enc.read()

## 6. Tell me what happens
