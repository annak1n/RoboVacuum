import numpy as np
try:
	import RPi.GPIO as GPIO

except RuntimeError:
	print("Error importing RPi.GPIO - need root ?")



# Wheel encoder class, there should only ever be one of these
# in the system

# one full wheel rotation is 2**10 (1024) clicks
class WheelEncoder: 

	# constructor 
	def __init__(self):
		GPIO.setmode(GPIO.BOARD)
		GPIO.setup(38, GPIO.OUT)
		GPIO.setup(40, GPIO.IN)
		GPIO.output(38,1)

	# delay()
	# simple busy loop delay
	def __delay(self, n):
		c = n
		while c > 0 :
			c=c-1


	# guard_pulse()
	# generate a long low pulse to signal to the 8051 that we
	# want to read the counter values
	def __guard_pulse(self):
		GPIO.output(38,0)
		self.__delay(1500)
		GPIO.output(38,1)
		self.__delay(1500)


	# read_16()
	# read 16 bits in from the 8051 
	# each bit is aligned to a clock we generate on GPIO 38

	def __read_16(self):
		res = 0
		for i in range (0, 16):
			GPIO.output(38,0)
			self.__delay(250)
			res = (res << 1) | GPIO.input(40)
			GPIO.output(38,1)
		return res
			

	# read_counters() 
	# Read 32 bits comprising two 16 bit counter values from the 8051.
	# Start by sending a guard pulse, then read the two 16 bit counter
	# values onee after the other

	def read_counters(self):
		self.__guard_pulse()
		counts = np.empty((2))	
		counts[0] = float(self.__read_16())
		counts[1] = float(self.__read_16())
		print(counts)
		return (counts)


# Example usage 
# wheel = WheelEncoder()
#
# i=1
# while i > 0:
#	print "%0.4x %0.4x" % wheel.read_counters() 
