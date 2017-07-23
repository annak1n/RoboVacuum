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
from control.pid import PID,GetAngleDifference
from accelerometer.BNO055 import BNO055
import ADC.IR_distance as DM
from threading import Semaphore
from math import cos, sin, pi, radians, isnan
from rotary_encoder import wheel_encoder as encoder
from math import pi, copysign, atan2
import wiringpi

from roboMath.movement import calc






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
            
        


        self.distanceSensor = DM.distanceMeas(calibrationFile='calibration_data.db')
        self.minDistance=7.5 #distance at which robot stops from wall
        self.distance=0
        self.speed = 0
        self.setAngle = 0.0
        self.RealAngle = self.bno.read_euler()[0]
        self.pid_angle = PID(I=.005, P=1.0, D=0, Angle=True)
        P=2
        I=0.5
        D=0.01
        self.pid_motors=[PID(P=P,I=I,D=D),PID(P=P,I=I,D=D)]
        self.rotEncode = encoder.WheelEncoder()
        self.bodyRadius = 32.0*0.5  # cm
        self.weight = 5.0

        self.MCfrequency=25.0 #motor control frequency in hertz
        self.INS=calc(2.65,27.0)

        #inter thread commincation variables
        self.sema=False #Lets all threads know that master thread is online (when true)
        self.stopDistance=False


      
        self.wheelSpeedsLinear=np.zeros(2)
        self.wheelSpeedsAngular=np.zeros(2)
        self.controlerWS=np.zeros(2)
        self.papirus = Papirus(rotation = 0)
        self.screen=PIL.Image.new("1",(self.papirus.width,self.papirus.height),"white")
        self.midScreen=np.array([self.papirus.width/2,self.papirus.height/2])
        
        
    def controlMotors(self, a):
        '''Function which is used to spawn the motor control thread

        '''
        set_dt=1.0/self.MCfrequency #the time step of one cycle
        dt=set_dt
        old_time=time.clock()
        wiringpi.delayMicroseconds(int((dt*1e6)))
        self.rotEncode.read_counters(np.zeros(2)) #this resets the encoders to zero to remove any initial errors
        time.sleep(0.01)
        while self.sema == True: #the sema allows the threads to be closed by another process              
             #get the x-y-phi rates of change from encoder, aswell as the wheel velocities
            #self.decodeSpeeds(dt)
            self.INS.rotWheels(np.copysign(self.rotEncode.read_counters(np.zeros(2)),self.controlerWS),dt)
            self.RealAngle = -self.bno.read_euler()[0] #get gyroscope angle
            
            self.controlerWS[0]=self.pid_motors[0].update(self.INS.dV[0])
            self.controlerWS[1]=self.pid_motors[1].update(self.INS.dV[1])
            self.distance=self.distanceSensor.getDistance()
            if isnan(self.distance):
                self.distance=120
            #self.INS.X[2]=np.arctan2(np.sin(self.INS.X[2]), np.cos(self.INS.X[2]))
            print(self.RealAngle, 180*self.INS.X[2]/pi)
            #print(self.INS.X)
            #print(dt)
            #print(self.wheelSpeeds)
            #print("c: " ,self.controlerWS,self.pid_motors[0].set_point,self.pid_motors[1].set_point)
            self.driveMotors.set_speed(self.controlerWS) #set the motor speed based upon the PID
            new_time=time.clock()
            sleep=int((set_dt-(new_time-old_time))*1e6)
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
        dm = 15#self.distanceSensor.getDistance()
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
        
         #run at 100 hertz


    def turnToAngle(self,angle):
        
        self.pid_angle.setPoint(angle)
        velocity=np.zeros(3)
        
        self.pid_angle.error=100
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
        self.guidence = thread.start_new_thread(self.controlMotors, (1,))
        #self.mapping = thread.start_new_thread(self.logDistance, (1,))
        #self.collision=thread.start_new_thread(self.collisionDetection,(1,))
        print("gidance thread initiated")
    
    def logDistance(self,a):
        dist_vect=np.zeros(2) 
        t=time.time()       
        while self.sema:
            time.sleep(0.05)
            dist_vect[0]=self.bodyRadius+self.distance
            c=cos(self.INS.X[2])
            s=sin(self.INS.X[2])
            R = np.zeros((2,2))
            R[0,0]=c
            R[1,0]=-s
            R[0,1]=s
            R[1,1]=c
            coord=self.midScreen+np.around(R.dot(dist_vect))
            if coord[0]>0 and coord[0] <174 and coord[1]>0 and coord[1]<164:
                self.screen.putpixel((int(coord[0]),int(coord[1])),0)
                #print(coord[0],coord[1])
            if (time.time()-t)>5:
              t=time.time()
              #self.papirus.display(self.screen)
              #self.papirus.update()
        self.papirus.display(self.screen)
        print("mapping done")
        self.papirus.update()

    def patternMove(self):
        angle=0
        delta=pi/2
        startpos=self.position
        while True:

          print("B")
          self.updateSpeedAngle(5,angle)
        
          print(self.distance)
          while self.distance >5.0:
            time.sleep(0.1)
          self.updateSpeedAngle(0,angle+delta)
          time.sleep(2)
          startpos=self.position[1:2]
          self.updateSpeedAngle(50,angle+delta)
          while np.linalg.norm(self.position[1:2]-startpos)<10 and self.distance>5:
            time.sleep(0.1)
          
          if abs(angle)<0.001:
              angle=pi
              delta=-pi/2
          else:
              angle=0
              delta=pi/2
          

    def stop(self):
        self.sema = False

    def startBrush(self):
        worker = thread.start_new_thread(
            self.brushMotors.set_speed, ([255, 255, 255],))

    def stopBrush(self):
        worker = thread.start_new_thread(
            self.brushMotors.set_speed, ([0, 0, 0],))

