import wpilib
import wpilib.drive
from wpilib import SmartDashboard as dash
from networktables import NetworkTables
import magicbot

from ctre import WPI_TalonSRX

import navx

from components.drivetrain import Drivetrain, Powertrain, Encoders
from components.navx_component import NavX

from utils import config_talon
from oi import DriverController, SysopController
from robotmap import RobotMap

class Robot(magicbot.MagicRobot):
    powertrain: Powertrain
    encoders: Encoders
    navx_component: NavX

    drivetrain_component: Drivetrain

    def createObjects(self):
        motor_object_left = [
            WPI_TalonSRX(port) for port in RobotMap.Drivetrain.motors_left
        ]
        self.motors_left = wpilib.SpeedControllerGroup(motor_object_left[0], *motor_object_left[1:])
        
        motor_object_right = [
            WPI_TalonSRX(port) for port in RobotMap.Drivetrain.motors_right
        ]
        self.motors_right = wpilib.SpeedControllerGroup(motor_object_right[0], *motor_object_right[1:])

        for motor in motor_object_left + motor_object_right:
            config_talon(motor, RobotMap.Drivetrain.motor_config)

        self.drivetrain = wpilib.drive.DifferentialDrive(self.motors_left, self.motors_right)
    
        self.encoder_left = WPI_TalonSRX(RobotMap.Encoders.left_encoder)
        self.encoder_right = WPI_TalonSRX(RobotMap.Encoders.right_encoder)

        self.navx = navx.AHRS.create_spi()

        self.driver = DriverController(wpilib.XboxController(0))
        self.sysop = SysopController(wpilib.XboxController(1))

    def reset_subsystems(self):
        self.drivetrain_component.reset()

    def teleopInit(self):
        self.reset_subsystems()

    def teleopPeriodic(self):
        driver_x, driver_y = self.driver.get_driver_input_curvature()
        self.drivetrain_component.curvature_drive(driver_y, driver_x)

        if self.driver.get_update_pid_pressed():
            self.drivetrain_component.update_pid_dash()

if __name__ == '__main__':
    wpilib.run(Robot)