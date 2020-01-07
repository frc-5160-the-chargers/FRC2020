from networktables import NetworkTables
import math

class Limelight:
    def __init__(self):
        self.reset()
        self.limelight_table = NetworkTables.getTable("limelight")

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

class MountedLimelight:
    def __init__(self, mounting_height, mounting_angle):
        self.limelight = Limelight()
        self.mounting_angle = mounting_angle
        self.mounting_height = mounting_height

    def reset(self):
        self.limelight.reset()

    def update(self):
        self.limelight.execute()

    def get_distance_trig(self, target_height):
        # TODO mounting angle, for now with just being a direct port we're assuming that they're mounted
        # on the same plane. this will (for obvious reasons) not always be the case

        target_angle = self.limelight.get_vertical_angle_offset()
        tan_angle = math.tan(math.radians(target_angle))
        height_difference = target_height-self.mounting_height
        
        if tan_angle != 0:
            return height_difference/tan_angle
        else:
            return 0