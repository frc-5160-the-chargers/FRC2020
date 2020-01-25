import wpilib
import wpilib.drive
from wpilib import SmartDashboard as dash
from networktables import NetworkTables
import magicbot

from ctre import WPI_TalonSRX

import navx

from components.drivetrain import Drivetrain, Powertrain, Encoders, DriveMode, EncoderSide
from components.navx_component import NavX

from utils import config_talon
from oi import DriverController, SysopController
from robotmap import RobotMap
from kinematics import ArcDrive
from dash import Tunable

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

        self.robot_controller = wpilib.RobotController()

        self.dash_target_angle = Tunable("Target Angle")
        self.dash_target_position = Tunable("Target Position")
        self.dash_v_max = Tunable("V_max")
        self.dash_arc_distance = Tunable("Arc Distance")
        self.dash_arc_radius = Tunable("Arc Radius")
        self.dash_target_velocity_left = Tunable("Target Velocity Left")
        self.dash_target_velocity_right = Tunable("Target Velocity Right")

        self.tunables = [
            self.dash_target_angle, self.dash_target_position, self.dash_v_max, self.dash_arc_distance, self.dash_arc_radius,
            self.dash_target_velocity_left, self.dash_target_velocity_right
        ]

    def reset_subsystems(self):
        self.drivetrain_component.reset()

    def teleopInit(self):
        self.reset_subsystems()

    def teleopPeriodic(self):
        # push all tunable values
        if self.driver.get_update_telemetry():
            for t in self.tunables:
                t.push()
            self.drivetrain_component.push_pid_dash()

        # update telemetry values
        dash.putNumber("NavX Angle", self.navx_component.get_angle())
        dash.putNumber("Drivetrain Position", self.drivetrain_component.get_position())
        dash.putNumber("Current Left Velocity", self.encoders.get_velocity(EncoderSide.LEFT))
        dash.putNumber("Current Right Velocity", self.encoders.get_velocity(EncoderSide.RIGHT))
        dash.putNumber("Battery Voltage", self.robot_controller.getBatteryVoltage())
        
        dash.putString("Current PID Mode", {
            DriveMode.PID_DRIVE: "PID Drive",
            DriveMode.PID_TURN: "PID Turn",
            DriveMode.VELOCITY_CONTROL: "Velocity Control"
        }[self.pid_mode])

        # pull all pid values off the dashboard
        if self.driver.get_update_pid_dash():
            self.drivetrain_component.update_pid_dash()

        # get the current arc for arc driving
        current_arc = ArcDrive(
            self.dash_arc_radius.get(),
            self.dash_arc_distance.get(),
            self.dash_v_max.get(),
            RobotMap.Drivetrain.wheelbase
        )

        # handle whatever the current pid controller is if starting pid
        if self.driver.get_enable_pid():
            if self.pid_mode == DriveMode.PID_TURN:
                self.drivetrain_component.turn_to_angle(self.dash_target_angle.get())
            if self.pid_mode == DriveMode.PID_DRIVE:
                self.drivetrain_component.drive_to_position(self.dash_target_position.get())
            if self.pid_mode == DriveMode.VELOCITY_CONTROL:
                self.drivetrain_component.velocity_control(self.dash_target_velocity_left.get(), self.dash_target_velocity_right.get())

        # drive the robot via commands
        if self.drivetrain_component.drive_mode == DriveMode.MANUAL_DRIVE:
            driver_x, driver_y = self.driver.get_driver_input_curvature()
            self.drivetrain_component.curvature_drive(driver_y, driver_x)
        elif self.drivetrain_component.drive_mode == DriveMode.PID_TURN:
            self.drivetrain_component.turn_to_angle(self.dash_target_angle.get())
        elif self.drivetrain_component.drive_mode == DriveMode.PID_DRIVE:
            self.drivetrain_component.drive_to_position(self.dash_target_position.get())
        elif self.drivetrain_component == DriveMode.VELOCITY_CONTROL:
            self.drivetrain_component.velocity_control(self.dash_target_velocity_left.get(), self.dash_target_velocity_right.get())

        # switch to manual control if enabled
        if self.driver.get_manual_control_override():
            self.drivetrain_component.drive_mode = DriveMode.MANUAL_DRIVE

        # rotate through pid control modes
        if self.driver.get_toggle_pid_type_pressed():
            self.pid_mode = {
                DriveMode.PID_TURN: DriveMode.PID_DRIVE,
                DriveMode.PID_DRIVE: DriveMode.VELOCITY_CONTROL,
                DriveMode.VELOCITY_CONTROL: DriveMode.PID_TURN
            }[self.pid_mode]

if __name__ == '__main__':
    wpilib.run(Robot)