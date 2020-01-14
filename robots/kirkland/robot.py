import wpilib
import wpilib.drive
from wpilib import SmartDashboard as dash
from networktables import NetworkTables
import magicbot

from ctre import WPI_TalonSRX

import navx

from components.drivetrain import Drivetrain, Powertrain, Encoders
from components.navx import NavX

from config import load_config_file

config_data = load_config_file()

subsystems_enabled = {
    "drivetrain": "drivetrain" in config_data['subsystems_enabled'],
}

class Robot(magicbot.MagicRobot):
    if subsystems_enabled['drivetrain']:
        powertrain: Powertrain
        encoders: Encoders
        navx_component: NavX

        drivetrain_component: Drivetrain

    def createObjects(self):
        if subsystems_enabled['drivetrain']:
            motor_object_left = [
                WPI_TalonSRX(port) for port in config_data['motor_ports']['drivetrain']['left']
            ]
            self.motors_left = wpilib.SpeedControllerGroup(motor_object_left[0], *motor_object_left[1:])
            
            motor_object_right = [
                WPI_TalonSRX(port) for port in config_data['motor_ports']['drivetrain']['right']
            ]
            self.motors_right = wpilib.SpeedControllerGroup(motor_object_right[0], *motor_object_right[1:])

            self.drivetrain = wpilib.drive.DifferentialDrive(self.motors_left, self.motors_right)
        
            self.encoder_left = WPI_TalonSRX(config_data['motor_ports']['drivetrain']['left_encoder'])
            self.encoder_right = WPI_TalonSRX(config_data['motor_ports']['drivetrain']['right_encoder'])

            self.navx = navx.AHRS.create_spi()

    def reset_subsystems(self):
        self.drivetrain_component.reset()

    def teleopInit(self):
        self.reset_subsystems()

    def teleopPeriodic(self):
        pass

if __name__ == '__main__':
    wpilib.run(Robot)