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
from  threading import Semaphore

class Robot(object):
    '''
    Class for containing all robot motore and sensing functions
    '''

    def __init__(self, driveMotors=MC.motor_control(0x61, [1, 3], [1, -1]), brushMotors=MC.motor_control(0x60, [2, 3, 4], [-1, 1, 1])):
        self.driveMotors = driveMotors
        self.brushMotors = brushMotors
        self.bno = BNO055(serial_port='/dev/ttyUSB0',rst=None)
        #self.bno.clockStretchBugMode(buffer_size=7)
        if self.bno.begin() is not True:
            print(str("Error initializing device"))
            exit()
        time.sleep(0.5)
        self.bno.getCalibrationFromFile(calibrationFile='calibration_data_A.db')
        self.distance = DM.distanceMeas(calibrationFile='calibration_data.db')
        self.speed = 0
        self.setAngle =0
        self.RealAngle = self.bno.read_euler()[0]
        self.pid = PID.PID(I=0.2, P=0.5,D=0.0,Angle=True)

    def setSpeedAngle(self,a):
        while self.sema==True:
            self.RealAngle=self.bno.read_euler()[0]
            err = self.pid.update(self.RealAngle)
            self.driveMotors.set_speed([self.speed, self.speed + err])
            time.sleep(0.01)
        self.driveMotors.set_speed([0,0])
        exit()
    
    def setSpeedAngleManual(self):
      self.RealAngle=self.bno.read_euler()[0]
      err = self.pid.update(self.RealAngle)
      print(err)
      self.driveMotors.set_speed([self.speed, self.speed + err])
      
    def updateSpeedAngle(self,setSpeed, setAngle):
        self.speed = setSpeed
        self.setAngle = setAngle
        self.pid.setPoint(self.setAngle)

    def updateDeltaSpeedAngle(self,setSpeed, setAngle):
        self.speed += setSpeed
        self.setAngle += setAngle
        if self.setAngle>360:
                self.setAngle-=360
        elif self.setAngle<0:
            self.setAngle+=360
        
            
        self.pid.setPoint(self.setAngle)
       

    def begin(self):
        print("starting guidance")
        self.sema=True
        print("starting guidance")
        self.guidence=thread.start_new_thread(self.setSpeedAngle,(1,))
        print("gidance thread initiated")

    def stop(self):
        self.sema=False


    def startBrush(self):
        worker = thread.start_new_thread(self.brushMotors.set_speed, ([255,255,255],))

    def stopBrush(self):
        worker = thread.start_new_thread(self.brushMotors.set_speed, ([0,0,0],))


