import time
import picamera
import numpy as np
from skimage import data
from skimage.exposure import rescale_intensity
import matplotlib.pyplot as plt


with picamera.PiCamera() as camera:
    #2592�1944
    camera.resolution = (2592, 1944)
    camera.framerate = 24
    time.sleep(2)
    output = np.empty((2592 * 1944 * 3,), dtype=np.uint8)
    camera.capture(output, 'rgb')
    output = output.reshape((2592, 1944, 3))
    #output = output[:100, :100, :]

#image = data.astronaut()

fig = plt.figure(figsize=(14, 7))
ax_each = fig.add_subplot(121, adjustable='box-forced')
ax_hsv = fig.add_subplot(122, sharex=ax_each, sharey=ax_each,
                         adjustable='box-forced')

# We use 1 - sobel_each(image)
# but this will not work if image is not normalized
ax_each.imshow(rescale_intensity(1 - sobel_each(output)))
ax_each.set_xticks([]), ax_each.set_yticks([])
ax_each.set_title("Sobel filter computed\n on individual RGB channels")

# We use 1 - sobel_hsv(image) but this will not work if image is not normalized
ax_hsv.imshow(rescale_intensity(1 - sobel_hsv(output)))
ax_hsv.set_xticks([]), ax_hsv.set_yticks([])
ax_hsv.set_title("Sobel filter computed\n on (V)alue converted image (HSV)")