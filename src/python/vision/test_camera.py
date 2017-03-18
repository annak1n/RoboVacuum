import time
import picamera
import numpy as np

with picamera.PiCamera() as camera:
    #2592×1944
    camera.resolution = (2592, 1944)
    camera.framerate = 24
    time.sleep(2)
    output = np.empty((2592 * 1944 * 3,), dtype=np.uint8)
    camera.capture(output, 'rgb')
    output = output.reshape((2592, 1944, 3))
    #output = output[:100, :100, :]