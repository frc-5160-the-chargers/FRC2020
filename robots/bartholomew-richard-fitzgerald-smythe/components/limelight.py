from networktables import NetworkTable

import math

import networktables

from robotmap import RobotMap

class LED_MODE:
    DEFAULT = 0;
    OFF = 1;
    BLINK = 2;
    ON = 3;

class Limelight:
    #limelight_table: NetworkTable

    def __init__(self):
        self.reset()
        self.mounting_angle = RobotMap.Limelight.mount_angle
        self.mounting_height = RobotMap.Limelight.mount_height
        self.FOV = 54; #degrees

    def get_horizontal_angle_offset(self):
        '''return the angle needed to turn to make the target in the center of view'''
        if self.valid_target:
            return self.horizontal_offset
        else:
            return 0

    def get_last_horizontal_angle_offset(self): #same as get_horizontal_angle_offset but defaults tot he most recent value and not zero
        return self.horizontal_offset

    def get_vertical_angle_offset(self):
        if self.valid_target:
            return self.vertical_offset
        else:
            return 0

    def get_last_vertical_angle_offset(self): #same as get_vertical_angle_offset but defaults tot he most recent value and not zero
        return self.vertical_offset
    
    def get_distance_trig(self, target_height): #only works when object is centered
        target_angle = self.get_vertical_angle_offset() + self.mounting_angle
        tan_angle = math.tan(math.radians(target_angle))
        height_difference = target_height-self.mounting_height
        
        if tan_angle != 0:
            return height_difference/tan_angle
        else:
            return 0

    def get_last_distance_trig(self, target_height): #only works when object is centered
        target_angle = self.get_last_vertical_angle_offset() + self.mounting_angle
        tan_angle = math.tan(math.radians(target_angle))
        height_difference = target_height-self.mounting_height
        
        if tan_angle != 0:
            return height_difference/tan_angle
        else:
            return 0

    def switch_pipeline(self,pipeline):
        self.limelight_table.putNumber('pipeline',pipeline);

    def switch_led_mode(self,mode):
        self.limelight_table.putNumber('ledMode',mode);
    
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