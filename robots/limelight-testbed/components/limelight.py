from networktables.networktable import NetworkTable

import math
import json

from config import load_config_file

config_data = load_config_file()

class Limelight:
    limelight_table: NetworkTable

    def __init__(self):
        self.reset()
        self.mounting_angle = config_data['limelight']['mount_angle']
        self.mounting_height = config_data['limelight']['mount_height']

    def get_horizontal_angle_offset(self):
        '''return the angle needed to turn to make the target in the center of view'''
        if self.valid_target:
            return self.horizontal_offset
        else:
            return 0

    def get_vertical_angle_offset(self):
        if self.valid_target:
            return self.vertical_offset
        else:
            return 0
    
    def get_distance_trig(self, target_height):
        target_angle = self.get_vertical_angle_offset() + self.mounting_angle
        tan_angle = math.tan(math.radians(target_angle))
        height_difference = target_height-self.mounting_height
        
        if tan_angle != 0:
            return height_difference/tan_angle
        else:
            return 0
    
    def reset(self):
        self.valid_target = False
        self.horizontal_offset = 0
        self.vertical_offset = 0
        self.target_area = 0

    def execute(self):
        targets = self.limelight_table.getNumber('tv', None)
        self.valid_target = targets >= 1.0 if targets != None else False

        if self.valid_target:
            self.horizontal_offset = self.limelight_table.getNumber('tx', None)
            self.vertical_offset = self.limelight_table.getNumber('ty', None)
            self.target_area = self.limelight_table.getNumber('ta', None)