from papirus import Papirus

from PIL import Image
from PIL import ImageFont
from PIL import ImageDraw 
from collections import deque
import time


'''
This module is for creating a display class which bufferes 
inputs to the display showing only the most recent and dropping
the rest. It has a refresh rate, at which it allows updates and 
once a new cycle comes around it will only print the most recent.

'''

class display(object):


    def __init__(self,refreshRate,message=none,frameH=164,frameW=176):
        self.refreshRate=refreshRate
        self.refreshInterval=1/refreshRate
        self.time=time.time()
        self.message=message
        self.frameH=frameH
        self.frameW=frameW
        self.queue= = deque(strftime("%H:%M:%S", gmtime())+str(": innit"),maxlen=round(frameH/12))
    
    def submit_info(self,message):
        self.queue.append(strftime("%H:%M:%S", gmtime())+str(message))

    def updateDisplay(self,message):
        
        font = ImageFont.truetype("sans-serif.ttf", 8)
        im = Image.new("RGB", (self.frameH, self.frameW), "white")

        popleft()