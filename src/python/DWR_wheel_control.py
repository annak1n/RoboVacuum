
from dimensions import ureg
from rotary_encoder import wheel_encoder as encoder
import motor_control.motor_class as MC
from filters.lag import lag_filter
from control.pid import PID, GetAngleDifference
from math import cos, sin, pi, radians, isnan,exp,sqrt
import numpy as np
import wiringpi
import time
import shelve
import thread
class pwm_input:
    def convert(self,v):
        return(v*6.27)

class wheel_control:

    def __init__(self,clicks_per_rotations= 1024.0, PID_values={'P':1.25,'I':0.75,'D':0.0025}, wheel_radius=3.0*ureg.cm,controler_frequency=50.0/ureg.second, pwm_input_converter = pwm_input):
        ''' Class to control the wheels of a DWR robot
        Each wheel has an independent PID controller
        The velocities of the wheels are determined using hub encoders, this signal is filtered to reduce noise from high sample rates
        '''

        #connect to HW
        self.rotEncode = encoder.WheelEncoder()
        self.driveMotors = MC.motor_group([0x61, 0x61], [1, 3], [1, -1])

        #create the rest
        self.frequency = controler_frequency
        self.time_step = 1.0/controler_frequency
        self.rad_per_click = (2*pi/clicks_per_rotations)*ureg.radians
        self.clicks_per_rad = (clicks_per_rotations/(2*pi))/ureg.radians
        self.wheel_radius=wheel_radius
        self.cm_per_click = self.rad_per_click*wheel_radius
        self.clicks_per_cm = 1/self.cm_per_click
        P = PID_values.get('P',0)
        I = PID_values.get('I',0)
        D = PID_values.get('D',0)
        self.pid_motors = [PID(P=P, I=I, D=D, unit = ureg.cm/ureg.seconds), PID(P=P, I=I, D=D, unit = ureg.cm/ureg.seconds)]
        self.clicks = np.zeros((2))
        self.encoder_values = lag_filter(self.time_step,0.5*ureg.second,0.95,np.zeros((2))*ureg.dimensionless)
        self.wheel_velocities_measured = np.zeros((2))
        self.wheel_velocities_controller = np.zeros((2))
        self.wheel_velocities_desired = np.zeros((2))
        temp = np.array([0,0])
        self.rotEncode.read_counters(temp)

        self.click_per_second = np.zeros((2))
        self.pwm_input_converter = pwm_input_converter

        self.non_stop=True

    def set_speed(self,wheel_velocities):
        '''Function used to set the speed called externally
        '''
        self.wheel_velocities_desired = wheel_velocities
        clicks_desired = self.clicks_per_cm * wheel_velocities
        self.pid_motors[0].setPoint(clicks_desired[0])
        self.pid_motors[1].setPoint(clicks_desired[1])

    
    def error_to_set_velocities(self):
        return (self.wheel_velocities_desired-self.wheel_velocities_measured)

    def measure_wheel_velocity(self):

        self.clicks = np.copysign(self.rotEncode.read_counters(self.clicks), self.wheel_velocities_desired)
        #print(self.clicks)
        self.encoder_values.update(self.clicks)
        self.click_per_second = self.encoder_values.value()/self.time_step
        self.wheel_velocities_measured = self.encoder_values.value()*self.cm_per_click*self.frequency
    
    def update_pid(self):
        self.wheel_velocities_controller[0] = self.pid_motors[0].update(
            self.encoder_values.value()[0])
        self.wheel_velocities_controller[1] = self.pid_motors[1].update(
            self.encoder_values.value()[1])

    def set_wheel_velocties(self):
        self.driveMotors.set_speed(pwm_input_converter.convert(self.controlerWS.to('cm/s').magnitude))


    def run_motor_control(self,A):
        dt = self.time_step
        
        while self.non_stop:
            interval = (time.time()*ureg.seconds + dt)
            self.measure_wheel_velocity()
            self.update_pid()
            self.set_wheel_velocties()
            delay = int( (interval - time.time()*ureg.seconds).to('microsecond').magnitude)  
            if delay > 0:
                wiringpi.delayMicroseconds( delay )

    def stop(self):
        self.non_stop=False

    def self_calibrate(self,steps=5):

        direction=1

        speeds = np.linspace(0,255,num=steps)
        counts= np.zeros((2,steps))
        refresh = np.zeros((2,steps))
        #self.guidence = thread.start_new_thread(self.run_motor_control, (1,))
        i=0
        for i in range(steps):
            self.driveMotors.set_speed([speeds[i]*direction,speeds[i]*direction])
            t1 = time.time()
            self.rotEncode.read_counters(np.array([0,0]))
            while time.time() < t1+3:
                if time.time()>t1+1:
                    time.sleep(0.02)
                    self.measure_wheel_velocity()
                    counts[0,i]+= (self.click_per_second[0]).to('1/s').magnitude
                    refresh[0,i]+=1
                    counts[1,i]+= (self.click_per_second[1]).to('1/s').magnitude
                    refresh[1,i]+=1

            #direction *=-1
        self.driveMotors.set_speed([0,0])

        print(counts/refresh)







