#!/usr/bin/python

import smbus
import time


"""
Helper functions used throughout the classes.
"""
def mapValues(vx, v1, v2, n1, n2):
    # v1 start of range, v2 end of range, vx the starting number
    percentage = (vx - v1) / (v2 - v1)
    # n1 start of new range, n2 end of new range
    return (n2 - n1) * percentage + n1


def constrain(value, minVal, maxVal):
    if value < minVal:
        value = minVal
    if value > maxVal:
        value = maxVal
    return value




class PCA9685:

    """
    Class to intialize and control the PCA9685 PWM generator, including setting
    voltages in form of percentages or driving servos.
    """

    def __init__(self, address=0x40):
        self.bus = smbus.SMBus(0)    # /dev/i2c-0
        self.address = address
        self.pins = [14, 13, 12, 15, 0, 1, 6, 7]    # P0-P7
        self.frequency = 200    # Default with 25Mhz oscillator
        self.startConfig()

    def writePWM(self, pin, percentage):
        percentage = constrain(percentage, 0, 100)
        percentage = int(percentage * 40.95)     # 0-100% -> 0-4095
        self.writeValues(pin, percentage)

    def writeServo(self, pin, angle):
        angle = constrain(angle, 0, 180)
        # Limit values at 50Hz, found experimentally
        self.servoMin = 128.0
        self.servoMax = 512.0
        scale = self.frequency / 50.0
        value = int(mapValues(angle, 0.0, 180.0, self.servoMin*scale,
                    self.servoMax*scale))
        self.writeValues(pin, value)

    def setFrequency(self, frequency):
        # Set SLEEP bit to 1 first
        conf = self.bus.read_byte_data(self.address, 0x00)
        conf = (conf & 0xEF) | 0x10
        self.bus.write_byte_data(self.address, 0x00, conf)
        # Calculate prescale from datasheet
        frequency = constrain(frequency, 24, 1526)
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




class ADS1000:

    """
    Class to intialize and read battery levels from the ADS1000 ADC
    """

    def __init__(self, address=0x48):
        self.bus = smbus.SMBus(0)    # /dev/i2c-0
        self.address = address

    def readBattery(self):
        rawData = self.bus.read_i2c_block_data(self.address, 0x00, 2)
        # From datasheet, 12bit ADC: 4 last bits of first byte and second byte
        adcH = rawData[0] & 15                 # Take lower 4 bits
        adc = adcH << 8 | rawData[1]           # Create 12bit word
        adc = self.twos_complement(adc, 12)    # Find decimal equivalent
        voltage = (1.625-adc*3.25/2048)*2      # 3V3 is actually at 3.25V, YMMV
        percent = mapValues(voltage, 3.3, 4.04, 0, 100)
        if percent > 100:                      # Max voltage when not charging
            percent = 100
        return (voltage, percent)

    def twos_complement(self, input_value, num_bits):
        mask = 2**(num_bits - 1)
        return -(input_value & mask) + (input_value & ~mask)




class DifferentialDrive:

    """
    Class to take high-level orders of robot speed and direction and translate
    to wheel speeds to individual PWM settings using the PCA9685 class.
    """

    def __init__(self, MotL1, MotL2, MotR1, MotR2, speedMin=10, speedMax=100,
                 address=0x40):
        self.pca9685 = PCA9685(address)
        self.motL1 = MotL1
        self.motL2 = MotL2
        self.motR1 = MotR1
        self.motR2 = MotR2
        self.maxSpeed = speedMax
        self.minSpeed = speedMin

    def drive(self, steering, throttle, mode=0):
        delay = 0.005
        if mode == 1 and throttle < 0:
            motATS = constrain(throttle * (1.0 - steering), -1.0, 1.0)
            motBTS = constrain(throttle * (1.0 + steering), -1.0, 1.0)
        else:
            motATS = constrain(throttle * (1.0 + steering), -1.0, 1.0)
            motBTS = constrain(throttle * (1.0 - steering), -1.0, 1.0)
        if mode == 1:
            # motAS = + steering * (1.0 - abs(throttle))
            # motBS = - steering * (1.0 - abs(throttle))
            motAS = + steering * (0.1 - abs(throttle)*0.1)
            motBS = - steering * (0.1 - abs(throttle)*0.1)
        else:
            motAS = 0.0
            motBS = 0.0
        motA = constrain(motATS+motAS, -1.0, 1.0)
        motB = constrain(motBTS+motBS, -1.0, 1.0)
        # print ("MotA: {:.2f} MotB: {:.2f}".format(motA, motB))
        if motA > 0:
            self.pca9685.writePWM(self.motL1, mapValues(motA, 0.0, 1.0,
                                  self.minSpeed, self.maxSpeed))
            time.sleep(delay)
            self.pca9685.writePWM(self.motL2, 0)
            time.sleep(delay)
        elif motA < 0:
            self.pca9685.writePWM(self.motL1, 0)
            time.sleep(delay)
            self.pca9685.writePWM(self.motL2, mapValues(abs(motA), 0.0, 1.0,
                                  self.minSpeed, self.maxSpeed))
            time.sleep(delay)
        else:
            self.pca9685.writePWM(self.motL1, 0)
            time.sleep(delay)
            self.pca9685.writePWM(self.motL2, 0)
            time.sleep(delay)

        if motB > 0:
            self.pca9685.writePWM(self.motR1, mapValues(motB, 0.0, 1.0,
                                  self.minSpeed, self.maxSpeed))
            time.sleep(delay)
            self.pca9685.writePWM(self.motR2, 0)
            time.sleep(delay)
        elif motB < 0:
            self.pca9685.writePWM(self.motR1, 0)
            time.sleep(delay)
            self.pca9685.writePWM(self.motR2, mapValues(abs(motB), 0.0, 1.0,
                                  self.minSpeed, self.maxSpeed))
            time.sleep(delay)
        else:
            self.pca9685.writePWM(self.motR1, 0)
            time.sleep(delay)
            self.pca9685.writePWM(self.motR2, 0)
            time.sleep(delay)
