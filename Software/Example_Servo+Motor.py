#!/usr/bin/python

import time
from NeoBoardClasses import PCA9685, mapValues

PCA9685_ADDRESS = 0x40
PCA9685 = PCA9685(PCA9685_ADDRESS)

PCA9685.setFrequency(50)

valMin = 0
valMax = 100
increment = 1
delay = 0.01

mot1 = 4
mot2 = 5
servo = 0

# Create list of speed values
speedValues = []
for speed in range(valMin, valMax, increment):
    speedValues.append(speed)
for speed in range(valMax, valMin, -increment):
    speedValues.append(speed)
for speed in range(valMin, -valMax, -increment):
    speedValues.append(speed)
for speed in range(-valMax, -valMin, increment):
    speedValues.append(speed)

# Iterate through different speeds
for speed in speedValues:
    print(speed)
    angle = mapValues(speed, float(valMin), float(valMax), 0.0, 180.0)
    if speed >= 0:
        PCA9685.writePWM(mot1, speed)
        PCA9685.writePWM(mot2, 0)
        PCA9685.writeServo(servo, angle)
    else:
        PCA9685.writePWM(mot1, 0)
        PCA9685.writePWM(mot2, -speed)
        PCA9685.writeServo(servo, -angle)
    time.sleep(delay)

# Turn off all outputs
PCA9685.writeAllOff()
