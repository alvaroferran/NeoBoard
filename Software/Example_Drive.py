#!/usr/bin/python

import time
from NeoBoardClasses import DifferentialDrive, PCA9685

motLA = 7
motLB = 6
motRA = 5
motRB = 4

wheels = DifferentialDrive(motLA, motLB, motRA, motRB)

delay = 1
# Steering, throttle
values = [[0.0, 0.5],
          [0.2, 0.5],
          [0.5, 0.5],
          [0.75, 0],
          [0, -0.1]
          ]

# Iterate through different speeds
for row in values:
    print(row)
    wheels.drive(row[0], row[1])
    time.sleep(delay)

# Turn off all outputs
wheels.pca9685.writeAllOff()
