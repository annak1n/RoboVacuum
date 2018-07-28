#!/usr/bin/python3

from ABE_ADCPi import ADCPi
from ABE_helpers import ABEHelpers
import time
import os

"""
================================================
ABElectronics ADC Pi 8-Channel ADC demo
Version 1.0 Created 29/02/2015

Requires python 3 smbus to be installed
run with: python3 demo-read_voltage.py
================================================


Initialise the ADC device using the default addresses and sample rate,
change this value if you have changed the address selection jumpers

Sample rate can be 12,14, 16 or 18
"""


i2c_helper = ABEHelpers()
bus = i2c_helper.get_smbus()
adc = ADCPi(bus, 0x69, 0x6B, 12)

while (True):

    # clear the console
    os.system('clear')

    # read from adc channels and print to screen
    print ("Channel 1: %02f" % adc.read_voltage(1))
    time.sleep(0.01)
    print ("Channel 2: %02f" % adc.read_voltage(2))
    time.sleep(0.01)
    print ("Channel 3: %02f" % adc.read_voltage(3))
    time.sleep(0.01)
    print ("Channel 4: %02f" % adc.read_voltage(4))
    time.sleep(0.01)
    print ("Channel 5: %02f" % adc.read_voltage(5))
    time.sleep(0.01)
    print ("Channel 6: %02f" % adc.read_voltage(6))
    time.sleep(0.01)
    print ("Channel 7: %02f" % adc.read_voltage(7))
    time.sleep(0.01)
    print ("Channel 8: %02f" % adc.read_voltage(8))
    print(adc.read_voltage(1))
    # wait 0.5 seconds before reading the pins again
    time.sleep(0.5)
