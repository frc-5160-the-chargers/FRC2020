import wpilib
from networktables import NetworkTables, NetworkTablesInstance
from wpilib import SmartDashboard as dash
import magicbot

from components.limelight import Limelight

from config import load_config_file

config_data = load_config_file()

class Robot(magicbot.MagicRobot):
    if "limelight" in config_data['enabled_devices']:
        limelight: Limelight

    def createObjects(self):
        pass

    def reset_subsystems(self):
        if "limelight" in config_data['enabled_devices']:
            self.limelight.reset()

    def teleopPeriodic(self):
        self.reset_subsystems()

        if "limelight" in config_data['enabled_devices']:
            dash.putNumber("Trig Calculated Distance: ", self.limelight.get_distance_trig(
                config_data['limelight']['target_height']
            ))

if __name__ == '__main__':
    wpilib.run(Robot)