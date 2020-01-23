import wpilib
import wpilib.drive
from wpilib import SmartDashboard as dash
from networktables import NetworkTables
import magicbot

from ctre import WPI_TalonSRX

import navx

from components.drivetrain import Drivetrain, Powertrain, Encoders, DriveMode
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
        self.drivetrain.setMaxOutput(RobotMap.Drivetrain.max_motor_power)
    
        self.encoder_left = WPI_TalonSRX(RobotMap.Encoders.left_encoder)
        self.encoder_right = WPI_TalonSRX(RobotMap.Encoders.right_encoder)

        self.navx = navx.AHRS.create_spi()

        self.driver = DriverController(wpilib.XboxController(0))
        self.sysop = SysopController(wpilib.XboxController(1))

        self.pid_mode = DriveMode.PID_TURN

    def reset_subsystems(self):
        self.drivetrain_component.reset()

    def teleopInit(self):
        self.reset_subsystems()

    def teleopPeriodic(self):
        if self.driver.get_update_telemetry():
            dash.putNumber("Target Angle", 0)
            dash.putNumber("Target Position", 0)
            self.drivetrain_component.push_pid_dash()

        if self.driver.get_toggle_pid_control():
            if self.pid_mode == DriveMode.PID_TURN:
                self.drivetrain_component.turn_to_angle(dash.getNumber("Target Angle", 0))
            if self.pid_mode == DriveMode.PID_DRIVE:
                self.drivetrain_component.drive_to_position(dash.getNumber("Target Position", 0))
        if self.driver.get_manual_control_override():
            self.drivetrain_component.drive_mode = DriveMode.MANUAL_DRIVE

        if self.driver.get_toggle_pid_type_pressed():
            self.pid_mode = {
                DriveMode.PID_TURN: DriveMode.PID_DRIVE,
                DriveMode.PID_DRIVE: DriveMode.PID_TURN
            }[self.pid_mode]

        if self.drivetrain_component.drive_mode == DriveMode.MANUAL_DRIVE:
            driver_x, driver_y = self.driver.get_driver_input_curvature()
            self.drivetrain_component.curvature_drive(driver_y, driver_x)
        elif self.drivetrain_component.drive_mode == DriveMode.PID_TURN:
            self.drivetrain_component.turn_to_angle(dash.getNumber("Target Angle", 0))
        elif self.drivetrain_component.drive_mode == DriveMode.PID_DRIVE:
            self.drivetrain_component.drive_to_position(dash.getNumber("Target Position", 0))

        dash.putNumber("NavX Angle", self.navx_component.get_angle())
        dash.putNumber("Drivetrain Position", self.drivetrain_component.get_position())

        dash.putString("Current PID Mode", {
            DriveMode.PID_DRIVE: "PID Drive",
            DriveMode.PID_TURN: "PID Turn"
        }[self.pid_mode])

        if self.driver.get_update_pid_pressed():
            self.drivetrain_component.update_pid_dash()

if __name__ == '__main__':
    wpilib.run(Robot)