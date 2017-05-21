import motor_control.motor_class as MC
import time
import os
# brush.set_speed([255,255])
import tty
import sys
import select
from papirus import PapirusText
import Queue
#from threading import Thread
import thread
import numpy as np
import control.pid as PID
from accelerometer.BNO055 import BNO055
import ADC.IR_distance as DM
from threading import Semaphore
from math import cos, sin, pi, radians
from rotary_encoder import wheel_encoder as encoder

class Robot(object):
    '''
    Class for containing all robot motore and sensing functions
    '''

    def __init__(self):
        '''Initialisation of the main robot controller
        - Connection to motor controllers
            -brushMotors
            -driveMotors
        - Connection to accelerometer BNO055
        - Connection to distance sensor
        '''
        self.driveMotors = MC.motor_group([0x61, 0x61], [1, 3], [1, -1])
        self.brushMotors = MC.motor_group([0x60, 0x60, 0X61], [2, 3, 4], [-1, 1, 1])
        self.bno = BNO055(serial_port='/dev/ttyUSB0', rst=None)
        # self.bno.clockStretchBugMode(buffer_size=7)
        if self.bno.begin() is not True:
            print(str("Error initializing device"))
            exit()
        time.sleep(0.5)
        self.bno.getCalibrationFromFile(
            calibrationFile='calibration_data_A.db')
        self.distance = DM.distanceMeas(calibrationFile='calibration_data.db')
        self.speed = 0
        self.setAngle = 0.0
        self.RealAngle = self.bno.read_euler()[0]
        self.pid_angle = PID.PID(I=0.2, P=0.5, D=0.0, Angle=True)
        self.pid_speed = PID.PID(I=0.2, P=0.5, D=0.0)
        self.theta = np.zeros(2)
        self.position = np.zeros(3)
        self.theta_dot = np.zeros(2)
        self.Jacobian = np.zeros((3, 2))
        self.rotEncode = encoder.WheelEncoder()
        self.bodyRadius = 30  # cm
        self.wheelRadius = 6  # cm need to check
        self.weight = 5
        self.Jacobian[2, 0] = 0.5 * self.bodyRadius / self.wheelRadius
        self.Jacobian[2, 1] = -self.Jacobian[2, 0]

    def setSpeedAngle(self, a):
        old_time=time.clock()
        enc=self.rotEncode.read_counters()
        time.sleep(0.01)
        while self.sema == True:
            enc1,enc2=self.rotEncode.read_counters()
            self.RealAngle = self.bno.read_euler()[0]
            err_angle = self.pid_angle.update(self.RealAngle)
            err_speed = self.pid_speed.update(enc1-enc2)
            self.driveMotors.set_speed([self.speed, self.speed + err_angle + err_speed])
            new_time=time.clock()
            time.sleep(0.02-(new_time-old_time))
            old_time=new_time
        self.driveMotors.set_speed([0, 0])
        exit()

    def setSpeedAngleManual(self):
        
        self.RealAngle = self.bno.read_euler()[0]
        err = self.pid_angle.update(self.RealAngle)
        print(err)
        self.driveMotors.set_speed([self.speed, self.speed + err])

    def updateSpeedAngle(self, setSpeed, setAngle):
        self.speed = setSpeed
        self.setAngle = setAngle
        self.pid_angle.setPoint(self.setAngle)

    def updateDeltaSpeedAngle(self, setSpeed, setAngle):
        self.speed += setSpeed
        self.setAngle += setAngle
        if self.setAngle > 360:
            self.setAngle -= 360
        elif self.setAngle < 0:
            self.setAngle += 360

        self.pid.setPoint(self.setAngle)

    def begin(self):
        print("starting guidance")
        self.sema = True
        print("starting guidance")
        self.guidence = thread.start_new_thread(self.setSpeedAngle, (1,))
        print("gidance thread initiated")

    def stop(self):
        self.sema = False

    def startBrush(self):
        worker = thread.start_new_thread(
            self.brushMotors.set_speed, ([255, 255, 255],))

    def stopBrush(self):
        worker = thread.start_new_thread(
            self.brushMotors.set_speed, ([0, 0, 0],))

    def calculateJacobian(self):
        ca = cos(radians(self.RealAngle))
        sa = sin(radians(self.RealAngle))
        self.Jacobian[0, :] = 0.5 * self.bodyRadius * ca
        self.Jacobian[1, :] = 0.5 * self.bodyRadius * sa
    
    def updatePosition(self,dS,dTheta):
        A=np.array([cos(self.RealAngle), 0],[sin(self.RealAngle), 0],[0, 1])
        self.position+=A.multiply([[dS],[dTheta]])