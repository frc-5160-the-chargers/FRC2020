import wpilib
from networktables import NetworkTables, NetworkTablesInstance
from wpilib import SmartDashboard as dash

from limelight import MountedLimelight

import json

import os

# load in config data/files
with open(f"{os.getcwd()}/data/json/config.json") as f:
    config_data = json.load(f)

class Robot(wpilib.TimedRobot):
    def robotInit(self):        
        self.limelight = MountedLimelight(
            config_data["mount_height"],
            config_data["mount_angle"]
        )

    def teleopPeriodic(self):
        self.limelight.update()

        dash.putNumber("Trig Calculated Distance: ", self.limelight.get_distance_trig(
            config_data["target_height"]
        ))

if __name__ == '__main__':
    wpilib.run(Robot)