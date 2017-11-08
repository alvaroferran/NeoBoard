#!/usr/bin/python

import time
from NeoBoardClasses import PCA9685, ADS1000

PCA9685_ADDRESS = 0x40
ADS1000_ADDRESS = 0x48

PCA9685 = PCA9685(PCA9685_ADDRESS)
ADS1000 = ADS1000(ADS1000_ADDRESS)

valMin = 0
valMax = 100
increment = 10
delay = 0.25

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

# Iterate though different speeds
for speed in speedValues:
    print(speed)
    if speed >= 0:
        PCA9685.writePWM(4, speed)
        PCA9685.writePWM(5, 0)
    else:
        PCA9685.writePWM(4, 0)
        PCA9685.writePWM(5, -speed)
    time.sleep(delay)

# Turn off all outputs
PCA9685.writeAllOff()

# Read battery level
voltage, percentage = ADS1000.readBattery()
print("Battery level is {:.2f}V, {:.2f}%".format(voltage, percentage))
