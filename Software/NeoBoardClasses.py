#!/usr/bin/python

import smbus
import time



def map(vx, v1, v2, n1, n2):
    # v1 start of range, v2 end of range, vx the starting number
    percentage = (vx - v1) / (v2 - v1)
    # n1 start of new range, n2 end of new range
    return (n2 - n1) * percentage + n1




class PCA9685:

    def __init__(self, PCA9685):
        self.bus = smbus.SMBus(0)    # /dev/i2c-0
        self.address = PCA9685
        self.pins = [14, 13, 12, 15, 0, 1, 6, 7]    # P0-P7
        self.frequency = 200    # Default with 25Mhz oscillator
        self.startConfig()

    def writePWM(self, pin, percentage):
        percentage = self.checkBoundaries(percentage, 0, 100)
        percentage = int(percentage * 40.95)     # 0-100% -> 0-4095
        self.writeValues(pin, percentage)

    def writeServo(self, pin, angle):
        angle = self.checkBoundaries(angle, 0, 180)
        # Limit values at 50Hz, found experimentally
        self.servoMin = 128.0
        self.servoMax = 512.0
        scale = self.frequency / 50.0
        value = int(map(angle, 0.0, 180.0, self.servoMin*scale, self.servoMax*scale))
        self.writeValues(pin, value)

    def setFrequency(self, frequency):
        # Set SLEEP bit to 1 first
        conf = self.bus.read_byte_data(self.address, 0x00)
        conf = (conf & 0xEF) | 0x10
        self.bus.write_byte_data(self.address, 0x00, conf)
        # Calculate prescale from datasheet
        frequency = self.checkBoundaries(frequency, 24, 1526)
        prescale = int(round(25000000.0/(4096*frequency))-1)
        self.bus.write_byte_data(self.address, 0xFE, prescale)
        # Actual frequency due to rounding errors
        self.frequency = 25000000/(4096*(prescale+1))
        # Reset chip
        self.startConfig()

    def writeAllOff(self):
        # Turn all outputs to LOW
        off = [0x00, 0x00, 0x00, 0x10]
        self.bus.write_i2c_block_data(self.address, 0xFA, off)

    def writeValues(self, index, value):
        regH = (value & 0xF00) >> 8
        regL = value & 0xFF
        vals = [0x00, 0x00, regL, regH]   # LedOnL, LedOnH, LedOffL, LedOffH
        channel = self.pins[index]
        self.bus.write_i2c_block_data(self.address, 0x06+channel*4, vals)

    def startConfig(self):
        # Set SLEEP to 0 to start oscillator
        self.bus.write_byte_data(self.address, 0x00, 0x01)
        time.sleep(0.001)
        # Set AutoIncrement to 1
        self.bus.write_byte_data(self.address, 0x00, 0xA0)
        # Set OUTDRV and OUTNE0 to 1 in MODE2 register
        self.bus.write_byte_data(self.address, 0x01, 0x05)
        time.sleep(0.001)
        self.writeAllOff()

    def checkBoundaries(self, value, minVal, maxVal):
        if value < minVal:
            value = minVal
        if value > maxVal:
            value = maxVal
        return value




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
        percent = map(voltage, 3.3, 4.04, 0, 100)
        if percent > 100:                      # Max voltage when not charging
            percent = 100
        return (voltage, percent)

    def twos_complement(self, input_value, num_bits):
        mask = 2**(num_bits - 1)
        return -(input_value & mask) + (input_value & ~mask)
