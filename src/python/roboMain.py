from pint import UnitRegistry
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
from math import cos, sin, pi, radians, isnan
from rotary_encoder import wheel_encoder as encoder
from math import pi, copysign, atan2
import wiringpi
from filters.lag import lag_filter

from copy import copy


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
        self.ureg = UnitRegistry()
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
        self.pid_angle = PID(I=.005, P=1.0, D=0, Angle=True)
        P = 4.0*.6
        I = 0.5
        D = 0.01
        self.pid_motors = [PID(P=P, I=I, D=D, unit = self.ureg.cm/self.ureg.seconds), PID(P=P, I=I, D=D, unit = self.ureg.cm/self.ureg.seconds)]

        self.bodyRadius = (32.0/2.0)*self.ureg.cm  # cm
        self.wheelRadius = 3*self.ureg.cm  # cm need to check
        self.wheelCircumference = self.wheelRadius*2.0*pi
        self.wheel2wheel = 26.5*self.ureg.cm
        self.weight = 5.0*self.ureg.kg

        self.Wheel2RoboCsys = np.zeros((3, 2))
        '''
        self.Wheel2RoboCsys[0, 1] = 0.5*self.wheelRadius
        self.Wheel2RoboCsys[0, 0] = 0.5*self.wheelRadius
        self.Wheel2RoboCsys[2, 0] = 0.5 * \
            (self.wheelRadius/(0.5*self.wheel2wheel))
        self.Wheel2RoboCsys[2, 0] = -0.5 * \
            (self.wheelRadius/(0.5*self.wheel2wheel))
        '''
        self.RoboCsys2Wheel = np.zeros((2, 3))
        '''
        self.RoboCsys2Wheel[0, 0] = 1/self.wheelRadius
        self.RoboCsys2Wheel[1, 0] = 1/self.wheelRadius
        self.RoboCsys2Wheel[0, 2] = (0.5*self.wheel2wheel)/self.wheelRadius
        self.RoboCsys2Wheel[1, 2] = -(0.5*self.wheel2wheel)/self.wheelRadius
        '''
        self.velocity_desired = np.zeros(3)
        self.MCfrequency = 50.0/self.ureg.second  # motor control frequency in hertz
        # rotary encoder clicks per rotation stored as invert to speed calc time
        self.rad_per_click = (2*pi/1024.0)*self.ureg.radians
        self.rotEncode = encoder.WheelEncoder()
        

        # inter thread commincation variables
        # Lets all threads know that master thread is online (when true)
        self.sema = False
        self.stopDistance = False
        self.clicks = np.zeros(2)
        self.rotation = np.zeros((3, 3))
        self.position = np.zeros(3)
        self.location = np.array([200, 200])
        self.observations = np.zeros((400, 400))

        self.rotation[2, 2] = 1

        self.wheelSpeeds = np.zeros(2)
        self.encoder_values = lag_filter(1/self.MCfrequency,0.5*self.ureg.second,0.95,np.zeros((2))*self.ureg.dimensionless)
        self.controlerWS = np.zeros(2) * self.ureg.cm/self.ureg.seconds
        self.papirus = Papirus(rotation=0)
        self.screen = PIL.Image.new(
            "1", (self.papirus.width, self.papirus.height), "white")
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
        velocity = self.rotation.dot(self.Wheel2RoboCsys).dot(self.wheelSpeeds)
        # print(self.wheelSpeeds)
        # velocity[2]/=3.162
        self.position += dt*velocity
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
        self.position[:] = 0
        while self.sema == True:  # the sema allows the threads to be closed by another process
            # get the x-y-phi rates of change from encoder, aswell as the wheel velocities
            self.decodeSpeeds(dt)

            # get gyroscope angle
            self.RealAngle = (self.bno.read_euler()[0])*self.ureg.degree

            self.controlerWS[0] = self.pid_motors[0].update(
                self.wheelSpeeds[0])
            self.controlerWS[1] = self.pid_motors[1].update(
                self.wheelSpeeds[1])
            self.distance = self.distanceSensor.getDistance()
            if isnan(self.distance):
                self.distance = 120 
            else:
                self.distance*= self.ureg.cm
            
            # set the motor speed based upon the PID
            self.driveMotors.set_speed(self.controlerWS.to('cm/s').magnitude)
            new_time = time.clock() *self.ureg.seconds

            sleep = int((dt-(new_time-old_time)).to('microseconds').magnitude)
            if sleep > 0:
                wiringpi.delayMicroseconds(sleep)
            else:
                dt = new_time-old_time  # should this lag the dt can be adapted
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
        self.pid_angle.setPoint(self.setAngle)
        self.pid_angle.update(self.position[2])
        while abs(self.pid_angle.error) > pi/180:
            velocity[2] = self.pid_angle.update(self.position[2])
            temp2 = self.RoboCsys2Wheel.dot(velocity)
            self.pid_motors[0].setPoint(temp2[0])
            self.pid_motors[1].setPoint(temp2[1])
            print(self.pid_angle.error)
            wiringpi.delayMicroseconds(int((0.01)*1e6))

        self.pid_motors[0].setPoint(setSpeed)
        self.pid_motors[1].setPoint(setSpeed)

    def turnToAngle(self, angle):

        self.pid_angle.setPoint(angle)
        velocity = np.zeros(3)

        self.pid_angle.error = 100
        self.RoboCsys2Wheel
        limit = (2*pi)/180
        while abs(self.pid_angle.error) > limit:
            print(180*self.position[2]/pi)
            velocity[2] = self.pid_angle.update(self.position[2])
            temp2 = self.RoboCsys2Wheel.dot(velocity)
            self.pid_motors[0].setPoint(temp2[0])
            self.pid_motors[1].setPoint(temp2[1])
            time.sleep(0.05)

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

    def stop(self):
        self.sema = False

    def startBrush(self):
        worker = thread.start_new_thread(
            self.brushMotors.set_speed, ([255, 255, 255],))

    def stopBrush(self):
        worker = thread.start_new_thread(
            self.brushMotors.set_speed, ([0, 0, 0],))
