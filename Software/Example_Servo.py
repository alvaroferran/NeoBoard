#!/usr/bin/python

import time
from NeoBoardClasses import PCA9685

PCA9685_ADDRESS = 0x40
PCA9685 = PCA9685(PCA9685_ADDRESS)

# Most servos work at 50Hz
PCA9685.setFrequency(50)

delay = 0.005

for angle in range(0, 180):
    PCA9685.writeServo(0, angle)
    print(angle)
    time.sleep(delay)

for angle in range(180, 0, -1):
    PCA9685.writeServo(0, angle)
    print(angle)
    time.sleep(delay)

# Turn off all outputs
PCA9685.writeAllOff()
