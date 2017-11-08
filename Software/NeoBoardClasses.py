#!/usr/bin/python

import smbus
import time


class PCA9685:

    def __init__(self, PCA9685):
        self.bus = smbus.SMBus(0)    # /dev/i2c-0
        self.address = PCA9685
        self.pins = [14, 13, 12, 15, 0, 1, 6, 7]    # P0-P7
        # Set SLEEP to 0 to start oscillator
        self.bus.write_byte_data(self.address, 0x00, 0x01)
        time.sleep(0.001)
        # Set AutoIncrement to 1
        self.bus.write_byte_data(self.address, 0x00, 0xA0)
        # Set OUTDRV and OUTNE0 to 1 in MODE2 register
        self.bus.write_byte_data(self.address, 0x01, 0x05)
        time.sleep(0.001)
        self.writeAllOff()

    def percent2value(self, percent):
        value = int(percent * 40.95)     # 0-100% -> 0-4095
        regH = (value & 0xF00) >> 8
        regL = value & 0xFF
        return (regL, regH)

    def writePWM(self, index, percentage):
        if percentage < 0:
            percentage = 0
        if percentage > 100:
            percentage = 100
        valL, valH = self.percent2value(percentage)
        vals = [0x00, 0x00, valL, valH]   # LedOnL, LedOnH, LedOffL, LedOffH
        channel = self.pins[index]
        self.bus.write_i2c_block_data(self.address, 0x06+channel*4, vals)

    def writeAllOff(self):
        # Turn all outputs to LOW
        off = [0x00, 0x00, 0x00, 0x10]
        self.bus.write_i2c_block_data(self.address, 0xFA, off)




class ADS1000:

    def __init__(self, ADS1000):
        self.bus = smbus.SMBus(0)    # /dev/i2c-0
        self.address = ADS1000

    def readBattery(self):
        rawData = self.bus.read_i2c_block_data(self.address, 0x00, 2)
        # From datasheet, 12bit ADC: 4 last bits of first byte and second byte
        adcH = rawData[0] & 15                 # Take lower 4 bits
        adc = adcH << 8 | rawData[1]           # Create 12bit word
        adc = self.twos_complement(adc, 12)    # Find decimal equivalent
        voltage = (1.625-adc*3.25/2048)*2      # 3V3 is actually at 3.25V, YMMV
        percent = self.map(voltage, 3.3, 4.2, 0, 100)
        return (voltage, percent)

    def map(self, vx, v1, v2, n1, n2):
        # v1 start of range, v2 end of range, vx the starting number
        percentage = (vx - v1) / (v2 - v1)
        # n1 start of new range, n2 end of new range
        return (n2 - n1) * percentage + n1

    def twos_complement(self, input_value, num_bits):
        mask = 2**(num_bits - 1)
        return -(input_value & mask) + (input_value & ~mask)
