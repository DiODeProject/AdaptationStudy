'''
    File name: totem.py
    Author: Francesco Canciani and Andreagiovanni Reina
            Sheffield Robotics, University of Sheffield, UK
    Date created: October 2018
    Date last modified: February 2019
'''
from msg import Msg

class Totem:
    def __init__(self, x, y, t_id, opinion, led, quality,appearance_time,disappearance_time,qualityAfterChange):
        self.id = t_id
        self.x = x
        self.y = y
        self.opinion = opinion
        self.led = led
        self.quality = quality
        self.qualityAfterChange = qualityAfterChange
        self.appearanceTime=appearance_time
        self.disappearanceTime=disappearance_time
        self.msg = Msg(self.id, self.opinion, self.quality, [self.x, self.y],self.disappearanceTime)

