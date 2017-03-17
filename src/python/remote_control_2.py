import motor_control.motor_class as MC
import time
import os
#brush.set_speed([255,255])
import tty
import sys
import select
from papirus import PapirusText
import Queue
#from threading import Thread
import thread

from accelerometer.BNO055 import BNO055 
import ADC.IR_distance as DM
def getch():
    try:
        import termios
    except ImportError:
        # Non-POSIX. Return msvcrt's (Windows') getch.
        import msvcrt
        return msvcrt.getch

    # POSIX system. Create and return a getch that manipulates the tty.
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


def go_forward(motors):
  motors.set_speed([255,255])
def go_backwards(motors):
  motors.set_speed([-100,-100]) 
def turn_left(motors):
  motors.set_speed([100,-100])
def turn_right(motors):
  motors.set_speed([-100,100])
def stop_motors(motors):
  motors.set_speed([0,0])
  

text = PapirusText()   
brush=MC.motor_control(0x60,[2,3,4],[-1,1,1])

motors=MC.motor_control(0x61,[1,3],[1,-1])
#bno = BNO055()
#if bno.begin() is not True:
#  print "Error initializing device"
#  exit()
time.sleep(1)

#bno.getCalibrationFromFile(calibrationFile='calibration_data_A.db')
dm=DM.distanceMeas(calibrationFile='calibration_data.db')     

while True:
  key = getch()
  
  time.sleep(0.015)
 # angle=bno.read_euler()
  distance=dm.getDistance()
  #worker=thread.start_new_thread(text.write,('Distance= '+str(distance),))
  os.system('cls' if os.name == 'nt' else 'clear')
 # print('angle=', angle[0],'distance= ',float(distance))      
  if key=='q':
    motors.set_speed([0,0])
    brush.set_speed([0,0,0])
    quit()
  elif key=='w':
    worker=thread.start_new_thread(go_forward,(motors,))
    #motors.set_speed([255,255])
  elif key=='a':
    worker=thread.start_new_thread(turn_left,(motors,))
    #motors.set_speed([100,-100])
  elif key=='d':
    worker=thread.start_new_thread(turn_right,(motors,))
    #motors.set_speed([-100,100])
  elif key=='s':
    worker=thread.start_new_thread(go_backwards,(motors,))
    motors.set_speed([-100,-100])
  elif key=='r':
    brush.set_speed([255,255,255])

  elif key=='e':
    motors.set_speed([0,0])
    brush.set_speed([0,0,0])
    worker=thread.start_new_thread(text.write,('Distance= '+str(distance),))
  else:
    motors.set_speed([0,0])
    
    