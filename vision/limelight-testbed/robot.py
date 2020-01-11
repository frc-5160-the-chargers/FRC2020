import wpilib
from networktables import NetworkTables, NetworkTablesInstance
from wpilib import SmartDashboard as dash
import magicbot

from rev import CANSparkMax, MotorType
from ctre import WPI_TalonSRX

import navx

from components.limelight import Limelight
from components.navx import NavX
from components.neo import Neo
from components.shooter import Shooter

from config import load_config_file

config_data = load_config_file()

class Robot(magicbot.MagicRobot):
    if "limelight" in config_data['enabled_devices']:
        limelight_component: Limelight
    
    if "navx" in config_data['enabled_devices']:
        navx_component: NavX

    if "shooter" in config_data['enabled_devices']:
        shooter_component: Shooter
    
    if "neo" in config_data['enabled_devices']:
        neo_component: Neo

    def createObjects(self):
        if "shooter" in config_data['enabled_devices']:
            self.shooter_srx_a = WPI_TalonSRX(1)
            self.shooter_srx_b = WPI_TalonSRX(2)
        
        if "neo" in config_data['enabled_devices']:
            self.neo_motor = CANSparkMax(3, MotorType.kBrushless)

        if "navx" in config_data['enabled_devices']:
            self.navx = navx.AHRS.create_spi()

        self.controller = wpilib.XboxController(0)

    def reset_subsystems(self):
        if "limelight" in config_data['enabled_devices']:
            self.limelight_component.reset()

        if "navx" in config_data['enabled_devices']:
            self.navx_component.reset()

        if "shooter" in config_data['enabled_devices']:
            self.shooter_component.reset()

        if "neo" in config_data['enabled_devices']:
            self.neo_component.reset()
        
    def teleopInit(self):
        self.reset_subsystems()

    def teleopPeriodic(self):
        if "shooter" in config_data['enabled_devices']:
            shooter_power = -self.controller.getY(hand=self.controller.Hand.kRight)
            if self.controller.getAButton():
                shooter_power = -1
            self.shooter_component.power = shooter_power

        if "neo" in config_data['enabled_devices']:
            neo_power = self.controller.getY(hand=self.controller.Hand.kLeft)
            self.neo_component.power = neo_power

        if "limelight" in config_data['enabled_devices']:
            dash.putNumber("Trig Calculated Distance: ", self.limelight_component.get_distance_trig(
                config_data['limelight']['target_height']
            ))

        if "navx" in config_data['enabled_devices']:
            dx, dy, dz = self.navx_component.get_displacement()
            if self.controller.getXButtonPressed():
                self.navx_component.reset_displacement()
            dash.putNumber("Displacement X", dx)
            dash.putNumber("Displacement Y", dy)
            dash.putNumber("Displacement Z", dz)

if __name__ == '__main__':
    wpilib.run(Robot)