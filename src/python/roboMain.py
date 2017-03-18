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
import control.pid as PID
from accelerometer.BNO055 import BNO055
import ADC.IR_distance as DM


class Robot(object):
    '''
    docstring for class
    '''

    def __init__(self, driveMotors=MC.motor_control(0x61, [1, 3], [1, -1]), brushMotors=MC.motor_control(0x60, [2, 3, 4], [-1, 1, 1])):
        self.driveMotors = driveMotors
        self.brushMotors = brushMotors
        self.bno = BNO055(i2c=True)
        self.bno.clockStretchBugMode(buffer_size=3)
        if bno.begin() is not True:
            print(str("Error initializing device"), end='\n')
            exit()
        time.sleep(0.5)
        bno.getCalibrationFromFile(calibrationFile='calibration_data_A.db')
        self.distance = DM.distanceMeas(calibrationFile='calibration_data.db')
        self.speed = 0
        self.angle = bno.readOrientationCS()
        self.pid = PID.PID(I=1, P=1)

    def setSpeedAngle(self, setSpeed, setAngle):
        self.speed = setSpeed
        self.angle = setAngle
        self.pid.set_point(self.angle)
        err = self.pid.update(bno.readOrientationCS())
        worker = thread.start_new_thread(self.driveMotors.set_speed, [
                                         self.speed, self.speed + err])
    def startBrush(self):
        worker = thread.start_new_thread(self.brushMotors.set_speed, [255,255,255])

    def stopBrush(self):
        worker = thread.start_new_thread(self.brushMotors.set_speed, [0,0,0])


