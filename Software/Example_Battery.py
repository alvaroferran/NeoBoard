#!/usr/bin/python

from NeoBoardClasses import ADS1000

ADS1000_ADDRESS = 0x48
ADS1000 = ADS1000(ADS1000_ADDRESS)

# Read battery level
voltage, percentage = ADS1000.readBattery()
print("Battery level is {:.2f}V, {:.2f}%".format(voltage, percentage))
