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
from control.pid import PID
from accelerometer.BNO055 import BNO055
import ADC.IR_distance as DM
from threading import Semaphore
from math import cos, sin, pi, radians, isnan
from rotary_encoder import wheel_encoder as encoder
from math import pi, copysign, atan2
import wiringpi








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
        self.brushMotors = MC.motor_group([0x60, 0x60, 0X61], [1, 3, 4], [-1, 1, 1])
        self.bno = BNO055(serial_port='/dev/ttyUSB0', rst=None)
        # self.bno.clockStretchBugMode(buffer_size=7)
        if self.bno.begin() is not True:
            print(str("Error initializing device"))
            exit()
        time.sleep(0.5)
        self.bno.getCalibrationFromFile(
            calibrationFile='calibration_data_A.db')
            
        
        self.distance = DM.distanceMeas(calibrationFile='calibration_data.db')
        self.minDistance=7.5 #distance at which robot stops from wall
        
        self.speed = 0
        self.setAngle = 0.0
        self.RealAngle = self.bno.read_euler()[0]
        self.OdoAngle=0
        self.pid_angle = PID(I=0, P=1, D=0.0, Angle=True)
        self.pid_motors=[PID(P=0.5,I=1,D=0),PID(P=0.5,I=1,D=0)]
        self.rotEncode = encoder.WheelEncoder()
        self.bodyRadius = 32/2  # cm
        self.wheelRadius = 3  # cm need to check
        self.wheelCircumference = self.wheelRadius*2*pi
        self.wheel2wheel = 26.5 
        self.weight = 5
        self.Wheel2RoboCsys=np.zeros((3,2))
        self.Wheel2RoboCsys[0,1]=0.5*self.wheelRadius
        self.Wheel2RoboCsys[0,0]=0.5*self.wheelRadius
        self.Wheel2RoboCsys[2,0]=0.5*(self.wheelRadius/(0.5*self.wheel2wheel))
        self.Wheel2RoboCsys[2,0]=-0.5*(self.wheelRadius/(0.5*self.wheel2wheel))
        self.RoboCsys2Wheel=np.zeros((2,3))
        self.RoboCsys2Wheel[0,0]=1/self.wheelRadius
        self.RoboCsys2Wheel[1,0]=1/self.wheelRadius
        self.RoboCsys2Wheel[0,2]=(0.5*self.wheel2wheel)/self.wheelRadius
        self.RoboCsys2Wheel[1,2]=-(0.5*self.wheel2wheel)/self.wheelRadius

        self.MCfrequency=100 #motor control frequency in hertz
        self.clickPerRotation=1/1024 #rotary encoder clicks per rotation stored as invert to speed calc time

        self.route=[]
        self.SegDistance=0
        self.distanceRST=False
        self.OdoAngle
        #inter thread commincation variables
        self.sema=False #Lets all threads know that master thread is online (when true)
        self.stopDistance=False
        self.clicks= np.zeros(2)
        self.rotation=np.zeros((3,3))
        self.position=np.zeros(3)
        self.rotation[2,2]=1

        self.wheelSpeeds=np.zeros(2)
        self.controlerWS=np.zeros(2)

    def decodeSpeeds(self,dt):
        '''Function for converting the encoder output to wheel velocities 
        '''
        
        self.controlerWS=np.zeros(2)
        self.clicks=np.copysign(self.rotEncode.read_counters(self.clicks),self.controlerWS)
        self.wheelSpeeds=self.clicks*self.clickPerRotation*self.wheelCircumference*self.MCfrequency
        c=cos(self.position[2])
        s=sin(self.position[2])
        self.rotation[0,0]=c
        self.rotation[1,1]=c
        self.rotation[1,0]=-s
        self.rotation[0,1]=s
        velocity = self.rotation.dot(self.Wheel2RoboCsys).dot(self.clicks)
        self.position+=dt*velocity

        
        
    def setSpeedAngle(self, a):
        '''FUnction which is used to spawn the motor control thread

        '''
        dt=1/self.MCfrequency
        old_time=time.clock()
        wiringpi.delayMicroseconds(int((dt*1e6)))
        time.sleep(0.5)
        self.decodeSpeeds(dt)
        time.sleep(0.01)
        while self.sema == True:
            if  self.distanceRST:
              self.SegDistance=0
              self.distanceRST=False
              
            self.decodeSpeeds(dt) #method for decoding the speeds
            
            self.RealAngle = self.bno.read_euler()[0]

            for i in range(2):
                self.controlerWS[i]=self.pid_motors[i].update(self.wheelSpeeds[i])

            self.driveMotors.set_speed(self.controlerWS)
            new_time=time.clock()
            wiringpi.delayMicroseconds(int((dt-(new_time-old_time))*1e6))
            old_time=new_time
        self.driveMotors.set_speed([0, 0])
        exit()

    def collisionDetection(self,a):
      while self.sema == True:
      
        '''The distance detection in loop, this sets a semaphore flag for the motors forward variable to be overwritten
        
        '''
        dm = 15#self.distance.getDistance()
        if isnan(dm)==False:
            if self.minDistance> dm:
              self.stopDistance=True
        
        if self.stopDistance==True:
          if isnan(dm):
            self.stopDistance=False
          elif dm>self.minDistance:
            self.stopDistance=False
              
        '''The error between the measured angle and the odometer angle provides an indication that robot has crashed
        '''
        print(self.OdoAngle,self.RealAngle)
        if abs(self.OdoAngle - self.RealAngle)>3:
          self.stopDistance=True
        
        wiringpi.delayMicroseconds(int((0.01)*1e6)) #run at 100 hertz
      

    def updateSpeedAngle(self, setSpeed, setAngle):
        self.pid_left.setPoint(setSpeed)
        self.pid_right.setPoint(setSpeed)
        self.setAngle = setAngle
        self.pid_angle.setPoint(self.setAngle)

    def turnToAngle(self,angle):
        
        self.pid_angle.setPoint(angle)
        velocity=np.zeros(3)
        error=100
        self.RoboCsys2Wheel
        limit=1/60
        while error>limit:
            temp=self.position
            error=temp[2]-angle
            velocity[2]=self.pid_angle.update(error)
            self.RoboCsys2Wheel.dot(velocity)
            

    def begin(self):
        print("starting guidance")
        self.sema = True
        print("starting guidance")
        self.guidence = thread.start_new_thread(self.setSpeedAngle, (1,))
        #self.collision=thread.start_new_thread(self.collisionDetection,(1,))
        print("gidance thread initiated")

        
    def patternMove(self):
        angle=0
        delta=90
        while True:
          self.distanceRST=True
          self.updateSpeedAngle(0,angle)
          time.sleep(2)
          self.distanceRST=True
          time.sleep(1)
          self.updateSpeedAngle(50,angle)
          dm = self.distance.getDistance()
          if isnan(dm):
            dm=120
          while dm >10:
            dm = self.distance.getDistance()
            if isnan(dm):
              dm=120
            time.sleep(0.01)
          self.updateSpeedAngle(0,angle+delta)
          time.sleep(2)
          self.distanceRST=True
          time.sleep(1)
          self.updateSpeedAngle(50,angle+delta)
          t=time.clock()
          dm=100
          while dm >10 and self.SegDistance<15:
            dm = self.distance.getDistance()
            #print(dm,time.clock()-t)
            if isnan(dm):
              dm=120
            time.sleep(0.01)
          
          if angle==0:
              angle=180
              delta=-90
          else:
              angle=0
              delta=90
          

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