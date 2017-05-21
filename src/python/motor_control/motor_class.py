import numpy as np
from scipy.optimize import minimize

from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor
import time
import atexit


class motor_device():
    '''A wrapped class for adafruits motor hat, this allows the motor to
    take a floating point inputs from -255 to +255. The class automatically
     changes the motor directions.
     The class also allows the direction of the motor to be flipped with a [-1 or +1] direction variable.

    '''

    def __init__(self, imh_adress, motor_id, direction):
        '''The class as an input requires the i2c adress of the motor
         controller and the motor numbers and direction of spin (+1 or -1)

        '''
        self.device = Adafruit_MotorHAT(addr=imh_adress)
        self.direction = direction
        self.motor = self.device.getMotor(motor_id)

    def setSpeed(self, speed):
        '''This method changes the speed, doing bounds checks for the maximum values
         and dealing with the motor inversion for negative speeds.

        '''
        speed *= self.direction
        if speed > 0.5:
            self.motor.run(Adafruit_MotorHAT.FORWARD)
            if speed > 255:
                speed = 255
            self.motor.setSpeed((abs(int(speed))))
        elif speed < -0.5:
            if speed < -255:
                speed = -255

            self.motor.run(Adafruit_MotorHAT.BACKWARD)
            self.motor.setSpeed((abs(int(speed))))
        else:
            self.motor.run(Adafruit_MotorHAT.RELEASE)
    @atexit.register
    def cleanUp(self):
        self.motor.run(Adafruit_MotorHAT.RELEASE)


class motor_group():
    '''This class is for using groups for motors potentially on different controllers

    '''

    def __init__(self, imh_adress, motor_id, direction):
        self.motor=[]
        for i in range(0, len(motor_id)):
            self.motor.append( motor_device(
                imh_adress[i], motor_id[i], direction[i]))

    def set_speed(self, speed):
        for i in range(len(speed)):
            self.motor[i].setSpeed(speed[i])
