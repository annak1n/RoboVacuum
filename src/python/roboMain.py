from dimensions import ureg
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
from control.pid import PID, GetAngleDifference
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
from roboMath import DWR_transformations
from DWR_wheel_control import  wheel_control
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
        self.distance = 0*self.ureg.cm
        self.speed = 0
        self.setAngle = 0.0
        self.RealAngle = (self.bno.read_euler()[0])*self.ureg.degree
        self.OdoAngle = 0
        self.pid_angle = PID(I=0.0, P=1.0, D=0, Angle=True,unit=self.ureg.radians)


        self.bodyRadius = (32.0/2.0)*self.ureg.cm  # cm
        self.wheelRadius = 3*self.ureg.cm  # cm need to check
        self.wheelCircumference = self.wheelRadius*2.0*pi
        self.wheel2wheel = 26.5*self.ureg.cm
        self.weight = 5.0*self.ureg.kg
        self.map = mapper((2000,2000),pixel_width=2.5*self.ureg.cm)
        self.position = np.array([1000,1000])
        ##map related
        self.observations=[]
        self.transformations = DWR_transformations(self.wheel2wheel,self.wheelRadius)
      

        self.MCfrequency = 50.0/self.ureg.second  # motor control frequency in hertz
        # rotary encoder clicks per rotation stored as invert to speed calc time
        
        

        # inter thread commincation variables
        # Lets all threads know that master thread is online (when true)
        self.sema = False
        self.stopDistance = False
        self.clicks = np.zeros(2)
        self.rotation = np.zeros((3, 3))
        self.wheel_controls = wheel_control()


        self.location = np.array([200, 200])


        self.rotation[2, 2] = 1
        self.robo_speed=0



        self.papirus = Papirus(rotation=0)
        self.screen = PIL.Image.new(
            "1", (self.papirus.width, self.papirus.height), "white")
        self.papirus.display(self.screen)
        self.papirus.update()
        self.midScreen = np.array(
            [self.papirus.width/2, self.papirus.height/2, 0])



    def sensors(self,A):
        sleep = (1/25)*1000000
        while self.sema == True:
            wiringpi.delayMicroseconds(sleep)
            #print(self.RealAngle)
            self.RealAngle = (self.bno.read_euler()[0])*ureg.degree
            test_distance = self.distanceSensor.getDistance()
            if isnan(test_distance):
                self.distance = 120 * ureg.cm
            else:
                self.distance= test_distance*ureg.cm

    def begin(self):
        self.sema = True
        print("starting guidance")
        self.guidence = thread.start_new_thread(self.wheel_controls.run_motor_control, (1,))
        self.sense = thread.start_new_thread(self.sensors, (1,))
        #self.startBrush()
        #self.mapping = thread.start_new_thread(self.logDistance, (1,))
        # self.collision=thread.start_new_thread(self.collisionDetection,(1,))
        print("gidance thread initiated")

    def turnToAngle(self,angle):
        self.wheel_controls.set_speed(np.array([60,-60])*ureg.cm/ureg.second)

        error = GetAngleDifference(self.RealAngle,angle)
        while error.to('degrees')>5.0*ureg.degrees:
            time.sleep(0.02)
            error = GetAngleDifference(self.RealAngle,angle)
            #print("cur angle",self.RealAngle)
        self.wheel_controls.set_speed([0,0]*ureg.cm/ureg.second)


    def random_run(self):

        direction = random.uniform(0,360)*self.ureg.degrees
        print('new_angle',direction)
        t1 = time.time()
        dist_flop = True
        speed = np.array([100,100])*self.ureg.cm/self.ureg.second
        spiral = False
        #self.turnToAngle(direction)
        self.wheel_controls.set_speed(speed)
        free_path = True
        path_time = time.time()
        while time.time()-t1 < 60*20:
            if self.distance > 10*self.ureg.cm:
                free_path = True
            else:
                free_path = False
            
            if time.time()>path_time+10:
                free_path = False
                

            if free_path == False:
                direction = self.RealAngle + random.uniform(90,270)*self.ureg.degrees
                if direction>(360*self.ureg.degrees):
                    direction -= 360*self.ureg.degrees
                self.wheel_controls.set_speed(np.array([0,0])*self.ureg.cm/self.ureg.second)
                time.sleep(0.25)
                self.turnToAngle(direction)
                time.sleep(0.25)
                path_time=time.time()
                
            else:
                free_path=True
                self.wheel_controls.set_speed(speed)
            if spiral:
                speed[1]=speed[0]*(time.time()-free_path)/10

            time.sleep(0.02)
    def stop(self):
        self.sema = False

    def startBrush(self):
        worker = thread.start_new_thread(
            self.brushMotors.set_speed, ([255, 255, 255],))

    def stopBrush(self):
        worker = thread.start_new_thread(
            self.brushMotors.set_speed, ([0, 0, 0],))

'''
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
            self.wheel_control.set_speedAngle(5, angle)

            print(self.distance)
            while self.distance > 5.0:
                time.sleep(0.1)
            self.wheel_control.set_speedAngle(0, angle+delta)
            time.sleep(2)
            startpos = self.position[1:2]
            self.wheel_control.set_speedAngle(50, angle+delta)
            while np.linalg.norm(self.position[1:2]-startpos) < 10 and self.distance > 5:
                time.sleep(0.1)

            if abs(angle) < 0.001:
                angle = pi
                delta = -pi/2
            else:
                angle = 0
                delta = pi/2
    '''





                



