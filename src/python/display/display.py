from papirus import Papirus

from PIL import Image
from PIL import ImageFont
from PIL import ImageDraw
from collections import deque
import time
#import sqlite3
from sqlite3worker import Sqlite3Worker
'''
This module is for creating a display class which bufferes 
inputs to the display showing only the most recent and dropping
the rest. It has a refresh rate, at which it allows updates and 
once a new cycle comes around it will only print the most recent.

'''
from enum import Enum


class message_type(Enum):
    text = 1
    image = 2
    point_cloud = 3


class Message(object):
    '''
    A object containing message information for the screen
    '''

    def __init__(self, data, type, tag, priority):
        self.data = data
        self.type = 2
        self.tag = 3


class display(object):

    def __init__(self, refreshRate, message=None, frameH=164, frameW=176, display_tags='All',dataBase='logging/message.db'):
        self.refreshRate = refreshRate
        self.refreshInterval = 1 / refreshRate
        self.time = time.time()
        self.frameH = frameH
        self.frameW = frameW
        self.dataBase=self.dataBase
        if os.path.exists(dataBase)== False:
            sql_worker = Sqlite3Worker(self.dataBase)
                    sql_worker.execute(
            '''CREATE TABLE Messages (time text, message text, type text, tag text, priority integer)''')
            sql_worker.close()
        else:
            

        self.db = sqlite3.connect('scratch/message_server.db')
        self.db.execute(
            '''CREATE TABLE Messages (time text, message text, type text, tag text, priority integer)''')
        self.db.commit()

    def submit_info(self, data, type, tag, priority):
        sql_worker = Sqlite3Worker(self.dataBase)
        sql_worker.execute(
            '''CREATE TABLE Messages (time text, message text, type text, tag text, priority integer)''')
        sql_worker.close()


    def updateDisplay(self, message):

        font = ImageFont.truetype("sans-serif.ttf", 8)
        im = Image.new("RGB", (self.frameH, self.frameW), "white")

        popleft()
