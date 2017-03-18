
#mpl.use('GTK')
import time
import picamera
import numpy as np
from skimage import data,io
from skimage.exposure import rescale_intensity
#from skimage.color.adapt_rgb import adapt_rgb, each_channel, hsv_value
from skimage import filters
import matplotlib as mpl
#@adapt_rgb(each_channel)
#def sobel_each(image):
#    return filters.sobel(image)
#import mpl.pyplot as plt


with picamera.PiCamera() as camera:

    camera.resolution = (2592, 1944)
    camera.framerate = 24
    time.sleep(2)
    output = np.empty((2592 * 1952 * 3,), dtype=np.uint8)
    camera.capture(output, 'rgb')
    output = output.reshape((2592, 1952, 3))
    #output = output[:100, :100, :]

#image = data.astronaut()

print("hello")
#fig = mpl.pyplot.figure(figsize=(14, 7))
#ax_each = fig.add_subplot(121, adjustable='box-forced')
#ax_hsv = fig.add_subplot(122, sharex=ax_each, sharey=ax_each,
#                        adjustable='box-forced')
print("figure?")
# We use 1 - sobel_each(image)
# but this will not work if image is not normalized
io.imsave(rescale_intensity(1 - filters.sobel(output[:][:][0])),'sobel1.jpg')
#ax_each.set_xticks([]), ax_each.set_yticks([])
#ax_each.set_title("Sobel filter computed\n on individual RGB channels")
#print("or sobel")
# We use 1 - sobel_hsv(image) but this will not work if image is not normalized
#ax_hsv.imshow(rescale_intensity(1 - sobel_hsv(output)))
#ax_hsv.set_xticks([]), ax_hsv.set_yticks([])
#ax_hsv.set_title("Sobel filter computed\n on (V)alue converted image (HSV)")