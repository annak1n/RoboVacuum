from rotary_encoder import wheel_encoder as encoder
import motor_control.motor_class as MC
import time


driveMotors = MC.motor_group([0x61, 0x61], [1, 3], [1, -1])

enc=encoder.WheelEncoder()
print(enc.read_counters())
driveMotors.set_speed([255, 255])
t1=time.time()
t2=t1

while t2<(t1+20):
  t2=time.time()
  print(enc.read_counters())
  time.sleep(.1)
driveMotors.set_speed([0, 0])