from my_pint import ureg
import motor_control.motor_class as MC
import time
import os
# brush.set_speed([255,255])
import tty
import sys
import select
from papirus import Papirus
import PIL
from PIL import ImageDraw
import Queue
#from threading import Thread
import thread
import numpy as np
from control.pid import PID
from accelerometer.BNO055 import BNO055
import ADC.IR_distance as DM
from threading import Semaphore
from math import cos, sin, pi, radians, isnan,exp,sqrt
from rotary_encoder import wheel_encoder as encoder
from math import pi, copysign, atan2
import wiringpi
from filters.lag import lag_filter
from math import exp,sqrt
from copy import copy
import random
from mapping import mapper, observation

def vel_2_pmw(v):
    return(v*6.27)



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
        self.ureg = ureg
        self.driveMotors = MC.motor_group([0x61, 0x61], [1, 3], [1, -1])
        self.brushMotors = MC.motor_group(
            [0x60, 0x60, 0X61], [1, 3, 4], [-1, 1, 1])
        self.bno = BNO055(serial_port='/dev/ttyUSB0', rst=None)
        # self.bno.clockStretchBugMode(buffer_size=7)
        if self.bno.begin() is not True:
            print(str("Error initializing device"))
            exit()
        time.sleep(0.5)
        self.bno.getCalibrationFromFile(
            calibrationFile='calibration_data_A.db')

        self.distanceSensor = DM.distanceMeas(
            calibrationFile='calibration_data.db')
        self.minDistance = 7.5  # distance at which robot stops from wall
        self.distance = 0
        self.speed = 0
        self.setAngle = 0.0
        self.RealAngle = self.bno.read_euler()[0]*self.ureg.degree
        self.OdoAngle = 0
        self.pid_angle = PID(I=.005, P=1.0, D=0, Angle=True,unit=self.ureg.radians)
        P = 1.25
        I = 0.075
        D = 0.0025
        self.pid_motors = [PID(P=P, I=I, D=D, unit = self.ureg.cm/self.ureg.seconds), PID(P=P, I=I, D=D, unit = self.ureg.cm/self.ureg.seconds)]

        self.bodyRadius = (32.0/2.0)*self.ureg.cm  # cm
        self.wheelRadius = 3*self.ureg.cm  # cm need to check
        self.wheelCircumference = self.wheelRadius*2.0*pi
        self.wheel2wheel = 26.5*self.ureg.cm
        self.weight = 5.0*self.ureg.kg
        self.map = mapper((2000,2000),pixel_width=2.5*self.ureg.cm)
        self.position = np.array([1000,1000])
        ##map related
        self.observations=[]

        self.Wheel2RoboCsys = np.zeros((3, 2))
        
        self.Wheel2RoboCsys[0, 1] = (0.5*self.wheelRadius).to('cm').magnitude
        self.Wheel2RoboCsys[0, 0] = (0.5*self.wheelRadius).to('cm').magnitude
        self.Wheel2RoboCsys[2, 0] = 0.5 * (self.wheelRadius/(0.5*self.wheel2wheel)).to('dimensionless').magnitude
        self.Wheel2RoboCsys[2, 0] = -0.5 * (self.wheelRadius/(0.5*self.wheel2wheel)).to('dimensionless').magnitude
        
        self.RoboCsys2Wheel = np.zeros((2, 3))

        self.RoboCsys2Wheel[0, 0] = 1/(self.wheelRadius.to('cm').magnitude)
        self.RoboCsys2Wheel[1, 0] = 1/(self.wheelRadius.to('cm').magnitude)
        self.RoboCsys2Wheel[0, 2] = ((0.5*self.wheel2wheel)/self.wheelRadius).to('dimensionless').magnitude
        self.RoboCsys2Wheel[1, 2] = (-(0.5*self.wheel2wheel)/self.wheelRadius).to('dimensionless').magnitude

        self.velocity_desired = np.zeros(3)
        self.MCfrequency = 25.0/self.ureg.second  # motor control frequency in hertz
        # rotary encoder clicks per rotation stored as invert to speed calc time
        self.rad_per_click = (2*pi/1024.0)*self.ureg.radians
        self.rotEncode = encoder.WheelEncoder()
        

        # inter thread commincation variables
        # Lets all threads know that master thread is online (when true)
        self.sema = False
        self.stopDistance = False
        self.clicks = np.zeros(2)
        self.rotation = np.zeros((3, 3))
        
        self.location = np.array([200, 200])


        self.rotation[2, 2] = 1
        self.robo_speed=0


        self.wheelSpeeds = np.zeros(2)
        self.encoder_values = lag_filter(1/self.MCfrequency,0.5*self.ureg.second,0.95,np.zeros((2))*self.ureg.dimensionless)
        self.controlerWS = np.zeros(2) * self.ureg.cm/self.ureg.seconds
        self.papirus = Papirus(rotation=0)
        self.screen = PIL.Image.new(
            "1", (self.papirus.width, self.papirus.height), "white")
        self.papirus.display(self.screen)
        self.papirus.update()
        self.midScreen = np.array(
            [self.papirus.width/2, self.papirus.height/2, 0])

    def decodeSpeeds(self, dt):
        '''Function for converting the encoder output to wheel velocities 
        '''

        self.clicks = np.copysign(
            self.rotEncode.read_counters(self.clicks), self.controlerWS)
        self.encoder_values.update(self.clicks)

        # print(self.clicks)
        self.wheelSpeeds = self.encoder_values.value()*self.rad_per_click*self.wheelRadius*self.MCfrequency
        #print("e: ",self.wheelSpeeds)
        c = cos(self.RealAngle)
        s = sin(self.RealAngle)

        self.rotation[0, 0] = c
        self.rotation[1, 1] = c
        self.rotation[1, 0] = s
        self.rotation[0, 1] = -s
        R = self.rotation[0:2,0:2]
        self.robo_speed=np.mean(self.wheelSpeeds)
        vec = np.array([self.robo_speed.to('cm/s').magnitude,0])*(self.ureg.cm/self.ureg.s)
        velocity = np.dot( R, vec)
        self.position += dt*velocity
        
        # self.rotation.dot(self.Wheel2RoboCsys).dot(self.wheelSpeeds)
        # print(self.wheelSpeeds)
        # velocity[2]/=3.162
        #self.position += dt*velocity
        #self.position[2] = self.RealAngle
        # print(self.position)

    def setSpeedAngle(self, a):
        '''FUnction which is used to spawn the motor control thread

        '''
        dt = 1.0/self.MCfrequency  # the time step of one cycle
        old_time = time.clock()*self.ureg.seconds
        wiringpi.delayMicroseconds(int((dt.to('microsecond').magnitude)))
        # this resets the encoders to zero to remove any initial errors
        self.decodeSpeeds(dt)
        time.sleep(0.01)
        self.position *= 0
        while self.sema == True:  # the sema allows the threads to be closed by another process
            # get the x-y-phi rates of change from encoder, aswell as the wheel velocities
            self.decodeSpeeds(dt)
            
            # get gyroscope angle
            self.RealAngle = (self.bno.read_euler()[0])*self.ureg.degree
            self.distance = self.distanceSensor.getDistance()
            if isnan(self.distance):
                self.distance = 120 * self.ureg.cm
            else:
                self.distance*= self.ureg.cm
                self.map.update(observation(self.distance+self.bodyRadius,self.RealAngle,position=self.position))
                
            
            self.controlerWS[0] = self.pid_motors[0].update(
                self.wheelSpeeds[0])
            self.controlerWS[1] = self.pid_motors[1].update(
                self.wheelSpeeds[1])
            #print(self.wheelSpeeds.to('cm/s'),self.controlerWS)
            print(self.distance)
            # set the motor speed based upon the PID
            self.driveMotors.set_speed(vel_2_pmw(self.controlerWS.to('cm/s').magnitude))
            new_time = time.clock() *self.ureg.seconds
            #print("elapsed",new_time-old_time)
            sleep = int((dt-(new_time-old_time)).to('microseconds').magnitude)
            #print(sleep)
            if sleep > 0:
                wiringpi.delayMicroseconds(sleep)
                #dt = new_time-old_time  # should this lag the dt can be adapted
            old_time = new_time
        self.driveMotors.set_speed([0, 0])  # set motors to zero on exit
        exit()

    def collisionDetection(self, a):
        while self.sema == True:

            '''The distance detection in loop, this sets a semaphore flag for the motors forward variable to be overwritten

            '''
            dm = 15  # self.distanceSensor.getDistance()
            if isnan(dm) == False:
                if self.minDistance > dm:
                    self.stopDistance = True

            if self.stopDistance == True:
                if isnan(dm):
                    self.stopDistance = False
                elif dm > self.minDistance:
                    self.stopDistance = False

            '''The error between the measured angle and the odometer angle provides an indication that robot has crashed
        '''
            # print(self.OdoAngle,self.RealAngle)
            # if abs(self.OdoAngle - self.RealAngle)>3:
            #  self.stopDistance=True

            # run at 100 hertz

    def updateSpeedAngle(self, setSpeed, setAngle):

        velocity = np.zeros(3)
        self.pid_angle.setPoint(self.setAngle.to('radians'))
        self.pid_angle.update(self.RealAngle)
        while abs(self.pid_angle.error) > 5*self.ureg.degrees:
            #velocity[2] = self.pid_angle.update(self.position[2])
            temp2 = self.RoboCsys2Wheel.dot(velocity)
            self.pid_motors[0].setPoint(vel_2_pmw(temp2[0]))
            self.pid_motors[1].setPoint(vel_2_pmw(temp2[1]))
            print(self.pid_angle.error)
            wiringpi.delayMicroseconds(int((0.01)*1e6))

        self.pid_motors[0].setPoint(setSpeed)
        self.pid_motors[1].setPoint(setSpeed)


    def updateSpeed(self, speed):
        self.pid_motors[0].setPoint(vel_2_pmw(speed))
        self.pid_motors[1].setPoint(vel_2_pmw(speed))

    def turnToAngle(self, angle):

        self.pid_angle.setPoint(angle)
        velocity = np.zeros(3)
        self.pid_motors[0].setPoint(vel_2_pmw(5)*self.ureg.cm/self.ureg.seconds  )
        self.pid_motors[1].setPoint(vel_2_pmw(-5)*self.ureg.cm/self.ureg.seconds)
        self.pid_angle.error = 100
        limit = 5*self.ureg.degrees
        while abs(self.pid_angle.error) > limit:
            time.sleep(0.001)
            self.pid_angle.update(self.RealAngle)
        self.pid_motors[0].setPoint(0*self.ureg.cm/self.ureg.seconds  )
        self.pid_motors[1].setPoint(0*self.ureg.cm/self.ureg.seconds)
        return True

    def build_map(self):
        canvas = copy(self.screen)
        d = ImageDraw.Draw(canvas)
        li = self.midScreen+self.rotation.dot(np.array([15, 0, 0]))
        d.line(
        (self.midScreen[0], self.midScreen[1], li[0], li[1]), fill=0)
        for ob in self.observations:
            ob/=2.5*self.ureg.cm
            ob=np.round(ob) + self.midScreen
            if ob[0]>=0 and ob[0]< self.papirus.width and ob[1]>=0 and ob[1]< self.papirus.height:
                canvas.putpixel((int(ob[0]),int(ob[1])),0)
        self.papirus.display(canvas)
        self.papirus.update()
        print(len(self.observations))

    def begin(self):
        self.sema = True
        print("starting guidance")
        self.guidence = thread.start_new_thread(self.setSpeedAngle, (1,))
        #self.mapping = thread.start_new_thread(self.logDistance, (1,))
        # self.collision=thread.start_new_thread(self.collisionDetection,(1,))
        print("gidance thread initiated")

    def logDistance(self, a):
        dist_vect = np.zeros(3)
        t = time.time()
        while self.sema:

                # print(coord[0],coord[1])
            
            if (time.time()-t) > 5:
                print(self.RealAngle)
                t = time.time()
                canvas = copy(self.screen)
                d = ImageDraw.Draw(canvas)
                li = self.midScreen+self.rotation.dot(np.array([15, 0, 0]))
                d.line(
                    (self.midScreen[0], self.midScreen[1], li[0], li[1]), fill=0)
                self.papirus.display(canvas)
                self.papirus.update()
        self.papirus.display(self.screen)
        print("mapping done")
        self.papirus.update()

    def patternMove(self):
        angle = 0
        delta = pi/2
        startpos = self.position
        while True:

            print("B")
            self.updateSpeedAngle(5, angle)

            print(self.distance)
            while self.distance > 5.0:
                time.sleep(0.1)
            self.updateSpeedAngle(0, angle+delta)
            time.sleep(2)
            startpos = self.position[1:2]
            self.updateSpeedAngle(50, angle+delta)
            while np.linalg.norm(self.position[1:2]-startpos) < 10 and self.distance > 5:
                time.sleep(0.1)

            if abs(angle) < 0.001:
                angle = pi
                delta = -pi/2
            else:
                angle = 0
                delta = pi/2

    def random_run(self):

        direction = random.uniform(0,360)*self.ureg.degrees

        t1 = time.time()
        dist_flop = True
        speed = 25*self.ureg.cm/self.ureg.s
        X=self.turnToAngle(direction)
        while time.time()-t1 < 100:

            if self.distance < 10*self.ureg.cm:
                direction = random.uniform(0,360)*self.ureg.degrees
                self.turnToAngle(direction)
                if self.distance > 10*self.ureg.cm:
                    self.updateSpeed(speed)


                

    def stop(self):
        self.sema = False

    def startBrush(self):
        worker = thread.start_new_thread(
            self.brushMotors.set_speed, ([255, 255, 255],))

    def stopBrush(self):
        worker = thread.start_new_thread(
            self.brushMotors.set_speed, ([0, 0, 0],))
