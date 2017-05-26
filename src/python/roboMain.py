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



def calc_speed(clicks,dt):
      distance=(((float(clicks)/1024.0)*2.0*pi*3.0))
      return(distance/dt),distance
      



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
        self.minDistance=7.5 #dustance at which robot stops from wall
        
        self.speed = 0
        self.setAngle = 0.0
        self.RealAngle = self.bno.read_euler()[0]
        self.OdoAngle=0
        self.pid_angle = PID(I=0, P=15, D=0.0, Angle=True)
        self.pid_left=PID(P=0.5,I=1,D=0)
        self.pid_right=PID(P=0.5,I=1,D=0) 
        self.theta = np.zeros(2)
        self.position = np.zeros(3)
        self.theta_dot = np.zeros(2)
        self.Jacobian = np.zeros((3, 2))
        self.rotEncode = encoder.WheelEncoder()
        self.bodyRadius = 32  # cm
        self.wheelRadius = 3  # cm need to check
        self.wheel2wheel = 26.5 
        self.weight = 5
        self.Jacobian[2, 0] = 0.5 * self.bodyRadius / self.wheelRadius
        self.Jacobian[2, 1] = -self.Jacobian[2, 0]
        self.temp=0
        self.route=[]
        self.SegDistance=0
        self.distanceRST=False
        self.OdoAngle
        #inter thread commincation variables
        self.sema=False #Lets all threads know that master thread is online (when true)
        self.stopDistance=False
        
        
    def setSpeedAngle(self, a):
        '''FUnction which is used to spawn the motor control thread

        '''
        dt=0.01
        old_time=time.clock()
        wiringpi.delayMicroseconds(int((0.01)*1e6))
        s_left=+0
        s_right=+0
        self.driveMotors.set_speed([25, 25])
        time.sleep(0.5)
        enc1,enc2=self.rotEncode.read_counters()
        self.SegDistance=0
        while self.sema == True:
            if  self.distanceRST:
              self.SegDistance=0
              self.distanceRST=False
            enc1,enc2=self.rotEncode.read_counters()
            
            self.RealAngle = self.bno.read_euler()[0]
            err_angle = self.pid_angle.update(self.RealAngle)
            if abs(err_angle)>127:
              err_angle=copysign(127,err_angle)
            abs_speed_left,distance_left=calc_speed(enc2,dt)
            distance_left=copysign(distance_left,s_left)
            self.speed_left=copysign(abs_speed_left,s_left) #as the encoder has no knowledge of direction the sign of the pervious motor signal is used to determine direction
            
            abs_speed_right,distance_right=calc_speed(enc1,dt)
            distance_right=copysign(distance_right,s_right)
            self.OdoAngle+=180*((distance_right-distance_left)/self.wheel2wheel)/pi
            self.speed_right=copysign(abs_speed_right,s_right) #copysign(?
            if abs(self.pid_left.set_point)>0.01:
              s_left=self.pid_left.update(self.speed_left)
            else:
              s_left=0
            if abs(self.pid_right.set_point)>0.01:
              s_right=self.pid_right.update(self.speed_right)
            else:
              s_right=0
            self.SegDistance+=0.5*(distance_left+distance_right)
            #print(self.SegDistance)
            #print("angle: ", err_angle, s_right, s_left,enc1,enc2)
            self.driveMotors.set_speed([s_right-err_angle, s_left+err_angle])
            new_time=time.clock()
            wiringpi.delayMicroseconds(int((new_time-old_time)*1e6))
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



    def begin(self):
        print("starting guidance")
        self.sema = True
        print("starting guidance")
        self.guidence = thread.start_new_thread(self.setSpeedAngle, (1,))
        #self.collision=thread.start_new_thread(self.collisionDetection,(1,))
        print("gidance thread initiated")

    def here_to_there(self,B,speed):
 
        dist=np.linalg.norm(B)
        angle=180*atan2(B[1],B[0])/pi
        print(angle)
        self.updateSpeedAngle(0,angle)
        print(self.pid_angle.error)
        while abs(self.pid_angle.error)>5:
            print('mis-aligned')
            time.sleep(0.1)
        time.sleep(1)
        t = dist/speed
        print(self.pid_angle.error)
        self.updateSpeedAngle(speed,angle)
        time.sleep(t)
        print(t)
        self.updateSpeedAngle(0,0)
        time.sleep(1)
        return True
        
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