import wpilib
import wpilib.drive

from wpilib import SmartDashboard as dash

import magicbot

from rev import CANSparkMax, MotorType

import navx

from oi import Driver, Sysop
from utils import config_spark
from robotmap import RobotMap
from dash import Tunable

from components.drivetrain import Drivetrain, Powertrain, DrivetrainState, EncoderSide
from components.sensors import Encoders, NavX

class Robot(magicbot.MagicRobot):
    powertrain: Powertrain
    encoders: Encoders
    navx: NavX

    drivetrain: Drivetrain

    def createObjects(self):
        # initialize physical objects
        motor_objects_left = [
            CANSparkMax(port, MotorType.kBrushless) for port in RobotMap.Drivetrain.motors_left
        ]
        self.left_motors = wpilib.SpeedControllerGroup(motor_objects_left[0], *motor_objects_left[1:])

        motor_objects_right = [
            CANSparkMax(port, MotorType.kBrushless) for port in RobotMap.Drivetrain.motors_right
        ]
        self.right_motors = wpilib.SpeedControllerGroup(motor_objects_right[0], *motor_objects_right[1:])

        for motor in motor_objects_left + motor_objects_right:
            config_spark(motor, RobotMap.Drivetrain.motor_config)

        self.differential_drive = wpilib.drive.DifferentialDrive(self.left_motors, self.right_motors)
        self.differential_drive.setMaxOutput(RobotMap.Drivetrain.max_motor_power)

        self.left_encoder = wpilib.Encoder(RobotMap.Encoders.left_encoder_a, RobotMap.Encoders.left_encoder_b)
        self.right_encoder = wpilib.Encoder(RobotMap.Encoders.right_encoder_a, RobotMap.Encoders.right_encoder_b)

        self.navx_ahrs = navx.AHRS.create_spi()

        self.driver = Driver(wpilib.XboxController(0))
        self.sysop = Sysop(wpilib.XboxController(1))
        
        self.rio_controller = wpilib.RobotController()

        # set up dashboard
        self.pid_mode = DrivetrainState.PID_TURNING

        self.dash_target_angle = Tunable("Target Angle")
        self.dash_target_position = Tunable("Target Position")
        self.dash_target_vel_left = Tunable("Target Velocity Left")
        self.dash_target_vel_right = Tunable("Target Velocity Right")

        self.tunables = [
            self.dash_target_angle,
            self.dash_target_position,
            self.dash_target_vel_left,
            self.dash_target_vel_right
        ]

    def reset_subsystems(self):
        self.drivetrain.reset()

    def teleopInit(self):
        self.reset_subsystems()

    def robotPeriodic(self):
        if self.isEnabled():
            # push all tunables
            if self.driver.get_update_telemetry():
                for t in self.tunables:
                    t.push()
                self.drivetrain.pid_manager.push_to_dash()
            
            # update tunables
            dash.putNumber("NavX Heading", self.navx.get_heading())
            dash.putNumber("Drivetrain Position", self.drivetrain.encoders.get_position(EncoderSide.BOTH))
            dash.putNumber("Left Velocity", self.drivetrain.encoders.get_velocity(EncoderSide.LEFT))
            dash.putNumber("Right Velocity", self.drivetrain.encoders.get_velocity(EncoderSide.RIGHT))
            dash.putNumber("Voltage", self.rio_controller.getBatteryVoltage())

            dash.putString("PID Mode", {
                DrivetrainState.PID_STRAIGHT: "Straight",
                DrivetrainState.PID_TURNING: "Turning",
                DrivetrainState.PID_VELOCITY: "Velocity"
            }[self.pid_mode])

    def teleopPeriodic(self):
        # drive the drivetrain as needed
        driver_x, driver_y = self.driver.get_curvature_output()

        # manually handled driving
        if self.drivetrain.state == DrivetrainState.MANUAL_DRIVE:
            self.drivetrain.curvature_drive(driver_y, driver_x)

        # check and see if we need to activate driver assists
        if self.drivetrain.ready_straight_assist() and self.driver.ready_straight_assist():
            self.drivetrain.drive_straight(driver_y)
        elif self.drivetrain.state == DrivetrainState.AIDED_DRIVE_STRAIGHT:
            self.drivetrain.state = DrivetrainState.MANUAL_DRIVE

        # revert to manual control if enabled
        if self.driver.get_manual_control_override():
            self.drivetrain.state = DrivetrainState.MANUAL_DRIVE

if __name__ == '__main__':
    wpilib.run(Robot)