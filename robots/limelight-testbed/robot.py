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
from dash import put_tuple
from oi import DriverController, SysopController

config_data = load_config_file()

subsystems_enabled = {
    "limelight": "limelight" in config_data['enabled_devices'],
    "navx": "navx" in config_data['enabled_devices'],
    "shooter": "shooter" in config_data['enabled_devices'],
    "neo": "neo" in config_data['enabled_devices'],
    "camera": "camera" in config_data['enabled_devices']
}

class Robot(magicbot.MagicRobot):
    if subsystems_enabled['limelight']:
        limelight_component: Limelight
    
    if subsystems_enabled['navx']:
        navx_component: NavX

    if subsystems_enabled['shooter']:
        shooter_component: Shooter
    
    if subsystems_enabled['neo']:
        neo_component: Neo

    def createObjects(self):
        if subsystems_enabled['limelight']:
            self.limelight_table = NetworkTables.getTable("limelight")

        if subsystems_enabled['navx']:
            self.navx = navx.AHRS.create_spi()

        if subsystems_enabled['shooter']:
            self.shooter_srx_a = WPI_TalonSRX(1)
            self.shooter_srx_b = WPI_TalonSRX(2)
        
        if subsystems_enabled['neo']:
            self.neo_motor = CANSparkMax(3, MotorType.kBrushless)

        if subsystems_enabled['camera']:
            wpilib.CameraServer.launch()

        self.driver = DriverController(wpilib.XboxController(0))
        self.sysop = SysopController(wpilib.XboxController(1))

    def reset_subsystems(self):
        if subsystems_enabled["limelight"]:
            self.limelight_component.reset()

        if subsystems_enabled["navx"]:
            self.navx_component.reset()

        if subsystems_enabled["shooter"]:
            self.shooter_component.reset()

        if subsystems_enabled["neo"]:
            self.neo_component.reset()
        
    def teleopInit(self):
        self.reset_subsystems()

    def teleopPeriodic(self):
        if subsystems_enabled["limelight"]:
            dash.putNumber("Trig Calculated Distance: ", self.limelight_component.get_distance_trig(
                config_data['limelight']['target_height']
            ))

        if subsystems_enabled["navx"]:
            if self.driver.get_navx_reset():
                self.navx_component.reset()
            put_tuple("Displacement", self.navx_component.displacement, int)
            put_tuple("Velocity", self.navx_component.velocity, int)
            put_tuple("Acceleration", self.navx_component.acceleration, int)

        if subsystems_enabled["shooter"]:
            shooter_power = -self.driver.get_shooter_axis()
            if self.driver.get_shooter_full():
                shooter_power = -1
            self.shooter_component.power = shooter_power

        if subsystems_enabled["neo"]:
            neo_kP = dash.getNumber("Shooter kP", 0)
            neo_kF = dash.getNumber("Shooter kF", 0)
            self.neo_component.update_pid(neo_kP, neo_kF)
           
            neo_power = dash.getNumber("Target RPM", 0)
            self.neo_component.set_rpm(neo_power)

            if self.driver.get_update_telemetry():
                dash.putNumber("Target RPM", 0)
                dash.putNumber("Shooter kP", 0)
                dash.putNumber("Shooter kF", 0)
            
            dash.putNumber("Shooter RPM", self.neo_component.get_raw_velocity())


if __name__ == '__main__':
    wpilib.run(Robot)