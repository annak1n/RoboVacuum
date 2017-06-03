import motor_control.motor_class as MC
import time
import os
# brush.set_speed([255,255])
import tty
import sys
import select
from papirus import Papirus
import PIL
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
        self.dist=0
        self.speed = 0
        self.setAngle = 0.0
        self.RealAngle = self.bno.read_euler()[0]
        self.OdoAngle=0
        self.pid_angle = PID(I=.005, P=1.0, D=0.00005, Angle=True)
        self.pid_motors=[PID(P=0.0,I=0.5,D=0),PID(P=0.0,I=0.5,D=0)]
        self.rotEncode = encoder.WheelEncoder()
        self.bodyRadius = 32.0/2.0  # cm
        self.wheelRadius = 3  # cm need to check
        self.wheelCircumference = self.wheelRadius*2.0*pi
        self.wheel2wheel = 26.5 
        self.weight = 5.0
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

        self.MCfrequency=75.0 #motor control frequency in hertz
        self.clickPerRotation=1.0/1024.0 #rotary encoder clicks per rotation stored as invert to speed calc time


        #inter thread commincation variables
        self.sema=False #Lets all threads know that master thread is online (when true)
        self.stopDistance=False
        self.clicks= np.zeros(2)
        self.rotation=np.zeros((3,3))
        self.position=np.zeros(3)
        self.rotation[2,2]=1
      
        self.wheelSpeeds=np.zeros(2)
        self.controlerWS=np.zeros(2)
        self.papirus = Papirus(rotation = 0)
        self.screen=PIL.Image.new("1",(self.papirus.width,self.papirus.height),"white")
        self.midScreen=np.array([self.papirus.width/2,self.papirus.height/2,0])

    def decodeSpeeds(self,dt):
        '''Function for converting the encoder output to wheel velocities 
        '''
        
        
        self.clicks=np.copysign(self.rotEncode.read_counters(self.clicks),self.controlerWS)
      

        #print(self.clicks)
        self.wheelSpeeds=self.clicks*self.clickPerRotation*self.wheelCircumference*self.MCfrequency
        #print("e: ",self.wheelSpeeds)
        c=cos(self.position[2])
        s=sin(self.position[2])
        self.rotation[0,0]=c
        self.rotation[1,1]=c
        self.rotation[1,0]=-s
        self.rotation[0,1]=s
        velocity = self.rotation.dot(self.Wheel2RoboCsys).dot(self.wheelSpeeds)
        #print(self.wheelSpeeds)
        self.position+=dt*velocity
        if self.position[2]>2*pi:
          self.position-=2*pi
        elif self.position[2]<0:
          self.position+=2*pi
        #print(self.position)
        
        
    def setSpeedAngle(self, a):
        '''FUnction which is used to spawn the motor control thread

        '''
        dt=1.0/self.MCfrequency #the time step of one cycle
        old_time=time.clock()
        wiringpi.delayMicroseconds(int((dt*1e6)))
        self.decodeSpeeds(dt) #this resets the encoders to zero to remove any initial errors
        time.sleep(0.01)
        while self.sema == True: #the sema allows the threads to be closed by another process              
            self.decodeSpeeds(dt) #get the x-y-phi rates of change from encoder, aswell as the wheel velocities
            
            self.RealAngle = pi*self.bno.read_euler()[0]/180 #get gyroscope angle
            self.controlerWS[0]=self.pid_motors[0].update(self.wheelSpeeds[0])
            self.controlerWS[1]=self.pid_motors[1].update(self.wheelSpeeds[1])
            self.dist=self.distance.getDistance()
            print(self.RealAngle-self.position[2])
            #print(180*self.position[2]/pi)
            #print("c: " ,self.controlerWS,self.pid_motors[0].set_point,self.pid_motors[1].set_point)
            self.driveMotors.set_speed(self.controlerWS) #set the motor speed based upon the PID
            new_time=time.clock()
            sleep=int((dt-(new_time-old_time))*1e6)
            if sleep>0:
              wiringpi.delayMicroseconds(sleep)
            else:
              dt=new_time-old_time #should this lag the dt can be adapted
            old_time=new_time
        self.driveMotors.set_speed([0, 0]) #set motors to zero on exit
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
        self.pid_angle.error=100
        self.RoboCsys2Wheel
        limit=(2*pi)/180
        while abs(self.pid_angle.error)>limit:
            print(180*self.position[2]/pi)
            velocity[2]=self.pid_angle.update(self.position[2])
            temp2=self.RoboCsys2Wheel.dot(velocity)
            self.pid_motors[0].setPoint(temp2[0])
            self.pid_motors[1].setPoint(temp2[1])
            time.sleep(0.05)
            

    def begin(self):
        self.sema = True
        print("starting guidance")
        self.guidence = thread.start_new_thread(self.setSpeedAngle, (1,))
        #self.mapping = thread.start_new_thread(self.logDistance, (1,))
        #self.collision=thread.start_new_thread(self.collisionDetection,(1,))
        print("gidance thread initiated")
    
    def logDistance(self,a):
        dist_vect=np.zeros(3) 
        t=time.time()       
        while self.sema:
            time.sleep(0.05)
            dist_vect[0]=self.bodyRadius+self.dist
            coord=self.midScreen+np.around(self.rotation.dot(dist_vect))
            if coord[0]>0 and coord[0] <174 and coord[1]>0 and coord[1]<164:
                self.screen.putpixel((int(coord[0]),int(coord[1])),0)
                #print(coord[0],coord[1])
            if (time.time()-t)>5:
              t=time.time()
              self.papirus.display(self.screen)
              self.papirus.update()
        self.papirus.display(self.screen)
        print("mapping done")
        self.papirus.update()

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

