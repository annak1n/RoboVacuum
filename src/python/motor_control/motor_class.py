import numpy as np
from scipy.optimize import minimize

from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor
import time
class motor_control():
  def __init__(self,imh_adress,motor_ids,direction):
    self.device=Adafruit_MotorHAT(addr=imh_adress)
    self.motor={}
    self.direction=direction
    for i in range(0,len(motor_ids)):
      print(i)
      self.motor[i]=self.device.getMotor(motor_ids[i])
      
  def set_speed(self,speed):
    for i in range(len(speed)):
      #print(i)
      speed[i]*=self.direction[i]
      if speed[i]>0.1:
        self.motor[i].run(Adafruit_MotorHAT.FORWARD)
        if speed[i]>255:
          speed[i]=255
        self.motor[i].setSpeed((abs(int(speed[i]))))
      elif speed[i]<-0.1:
        if speed[i]<-255:
          speed[i]=-255
        
        self.motor[i].run(Adafruit_MotorHAT.BACKWARD)
        self.motor[i].setSpeed((abs(int(speed[i]))))
      else:
        self.motor[i].run(Adafruit_MotorHAT.RELEASE)