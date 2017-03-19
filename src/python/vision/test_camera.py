#mpl.use('GTK')
import time
import picamera
import numpy as np
from skimage import data
import scipy.misc
from skimage.exposure import rescale_intensity
#from skimage.color.adapt_rgb import adapt_rgb, each_channel, hsv_value
from skimage import filters
from skimage import feature

import matplotlib as mpl
#@adapt_rgb(each_channel)
#def sobel_each(image):
#    return filters.sobel(image)
#import mpl.pyplot as plt
im_size=4

with picamera.PiCamera() as camera:

    camera.resolution = (2592 , 1944 )
    camera.framerate = 15
    time.sleep(2)
    output = np.empty((2592 * 1952 * 3,), dtype=np.uint8)
    #output = np.empty((2592 * 1952 * 3,), dtype=np.uint8)
    camera.capture(output, 'rgb')
    #output = output.reshape((2592, 1952, 3))
    #output = output[:100, :100, :]
    output = output.reshape((1952, 2592, 3))
    output = output[:1952, :2592, :]

#image = data.astronaut()

print("hello")
#fig = mpl.pyplot.figure(figsize=(14, 7))
#ax_each = fig.add_subplot(121, adjustable='box-forced')
#ax_hsv = fig.add_subplot(122, sharex=ax_each, sharey=ax_each,
#                        adjustable='box-forced')
print("figure?")
output[:,:,0] = feature.canny(output[:,:,0])
output[:,:,1] = feature.canny(output[:,:,1])
output[:,:,2] = feature.canny(output[:,:,2])
scipy.misc.imsave('outfile.jpg', 255*output)
