import wpilib
import wpilib.drive

from wpilib import SmartDashboard as dash

import magicbot

from rev import CANSparkMax, MotorType
from ctre import WPI_TalonSRX

import navx

from oi import Driver, Sysop
from utils import config_spark, config_talon
from robotmap import RobotMap
from dash import Tunable

from components.drivetrain import Drivetrain, Powertrain, DrivetrainState, EncoderSide
from components.intake import IntakeLift, IntakeRoller, Intake
from components.sensors import Encoders, NavX

class Robot(magicbot.MagicRobot):
    powertrain: Powertrain
    encoders: Encoders
    navx: NavX

    drivetrain: Drivetrain

    intake_lift: IntakeLift
    intake_roller: IntakeRoller

    intake: Intake

    def createObjects(self):
        # initialize physical objects
        # drivetrain
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

        self.left_encoder = wpilib.Encoder(RobotMap.Encoders.left_encoder_b, RobotMap.Encoders.left_encoder_a)

        self.right_encoder = wpilib.Encoder(RobotMap.Encoders.right_encoder_b, RobotMap.Encoders.right_encoder_a)
        self.right_encoder.setReverseDirection(True)
        
        self.left_encoder.setDistancePerPulse(RobotMap.Encoders.distance_per_pulse)
        self.right_encoder.setDistancePerPulse(RobotMap.Encoders.distance_per_pulse)

        self.navx_ahrs = navx.AHRS.create_spi()

        # intake
        self.intake_lift_motor = WPI_TalonSRX(RobotMap.IntakeLift.motor_port)
        self.intake_lift_motor.configPeakOutputForward(RobotMap.IntakeLift.max_power)
        self.intake_lift_motor.configPeakOutputReverse(-RobotMap.IntakeLift.max_power)
        config_talon(self.intake_lift_motor, RobotMap.IntakeLift.motor_config)

        self.intake_roller_motor = WPI_TalonSRX(RobotMap.IntakeRoller.motor_port)
        self.intake_roller_motor.configPeakOutputForward(RobotMap.IntakeRoller.max_power)
        self.intake_roller_motor.configPeakOutputReverse(-RobotMap.IntakeRoller.max_power)
        config_talon(self.intake_roller_motor, RobotMap.IntakeRoller.motor_config)

        # self.intake_lift_encoder = wpilib.Encoder(RobotMap.IntakeLift.encoder_port_b, RobotMap.IntakeLift.encoder_port_a)
        # self.intake_lift_encoder.reset()
        # self.intake_lift_encoder.setSamplesToAverage(RobotMap.IntakeLift.encoder_averaging)
        # self.intake_lift_encoder.setDistancePerPulse(RobotMap.IntakeLift.encoder_distance_per_pulse)

        # controllers and electrical stuff
        self.driver = Driver(wpilib.XboxController(0))
        self.sysop = Sysop(wpilib.XboxController(1))
        
        # self.rio_controller = wpilib.RobotController()

        # set up dashboard
        self.pid_mode = DrivetrainState.PID_TURNING

        self.dash_target_angle = Tunable("Target Angle")
        self.dash_target_position = Tunable("Target Position")
        self.dash_target_vel_left = Tunable("Target Velocity Left")
        self.dash_target_vel_right = Tunable("Target Velocity Right")
        self.dash_target_lift_position = Tunable("Target Position Lift")

        self.tunables = [
            self.dash_target_angle,
            self.dash_target_position,
            self.dash_target_vel_left,
            self.dash_target_vel_right,
            self.dash_target_lift_position
        ]

    def reset_subsystems(self):
        self.drivetrain.reset()
        self.intake.reset()

    def teleopInit(self):
        pass

    def teleopPeriodic(self):
        # # drive the drivetrain as needed
        # driver_x, driver_y = self.driver.get_curvature_output()

        # # manually handled driving
        # if self.drivetrain.state == DrivetrainState.MANUAL_DRIVE:
        #     self.drivetrain.curvature_drive(driver_y, driver_x)

        # # check and see if we need to activate driver assists
        # # if self.drivetrain.ready_straight_assist() and self.driver.ready_straight_assist():
        # #     self.drivetrain.drive_straight(driver_y)
        # # elif self.drivetrain.state == DrivetrainState.AIDED_DRIVE_STRAIGHT:
        # #     self.drivetrain.state = DrivetrainState.MANUAL_DRIVE

        # # revert to manual control if enabled
        # if self.driver.get_manual_control_override():
        #     self.drivetrain.state = DrivetrainState.MANUAL_DRIVE

        # # handle intake control
        # # lift
        # intake_power = self.sysop.get_intake_lift_axis()
        # if intake_power > 0:
        #     self.intake_lift.raise_lift(intake_power)
        # elif intake_power < 0:
        #     self.intake_lift.lower_lift(intake_power)
        # else:
        #     self.intake_lift.stop()

        # # # intake rollers
        # if self.sysop.get_intake_intake():
        #     self.intake_roller.intake()
        # elif self.sysop.get_intake_outtake():
        #     self.intake_roller.outtake()
        # else:
        #     self.intake_roller.stop()



        # PID TUNING CODE (remove when done)
        # push tunables
        if self.driver.get_update_telemetry():
            for t in self.tunables:
                t.push()
            self.drivetrain.pid_manager.push_to_dash()
        
        # update tunables
        dash.putNumber("NavX Heading", self.navx.get_heading())
        dash.putNumber("Drivetrain Position", self.drivetrain.encoders.get_position(EncoderSide.BOTH))
        dash.putNumber("Left Velocity", self.drivetrain.encoders.get_velocity(EncoderSide.LEFT))
        dash.putNumber("Right Velocity", self.drivetrain.encoders.get_velocity(EncoderSide.RIGHT))

        dash.putString("PID Mode", {
            DrivetrainState.PID_STRAIGHT: "Straight",
            DrivetrainState.PID_TURNING: "Turning",
            DrivetrainState.PID_VELOCITY: "Velocity",
        }[self.pid_mode])

        # handle the current pid controller if starting a pid controller
        if self.driver.get_enable_pid():
            if self.pid_mode == DrivetrainState.PID_TURNING:
                self.drivetrain.turn_to_angle(self.dash_target_angle.get())
            if self.pid_mode == DrivetrainState.PID_STRAIGHT:
                self.drivetrain.drive_to_position(self.dash_target_position.get())
            if self.pid_mode == DrivetrainState.PID_VELOCITY:
                self.drivetrain.velocity_control(self.dash_target_vel_left.get(), self.dash_target_vel_right.get())

        # rotate through PID control types
        if self.driver.get_toggle_pid_type_pressed():
            self.pid_mode = {
                DrivetrainState.PID_STRAIGHT: DrivetrainState.PID_TURNING,
                DrivetrainState.PID_TURNING: DrivetrainState.PID_VELOCITY,
                DrivetrainState.PID_VELOCITY: DrivetrainState.PID_STRAIGHT
            }[self.pid_mode]

        # update values from dash if requested
        if self.driver.get_update_pid_dash():
            self.drivetrain.pid_manager.update_from_dash()

        if self.drivetrain.state == DrivetrainState.MANUAL_DRIVE:
            driver_x, driver_y = self.driver.get_curvature_output()
            self.drivetrain.curvature_drive(driver_y, driver_x)
        elif self.pid_mode == DrivetrainState.PID_TURNING:
            self.drivetrain.turn_to_angle(self.dash_target_angle.get())
        elif self.pid_mode == DrivetrainState.PID_STRAIGHT:
            self.drivetrain.drive_to_position(self.dash_target_position.get())
        elif self.pid_mode == DrivetrainState.PID_VELOCITY:
            self.drivetrain.velocity_control(self.dash_target_vel_left.get(), self.dash_target_vel_right.get())

        # revert to manual control if enabled
        if self.driver.get_manual_control_override():
            self.drivetrain.state = DrivetrainState.MANUAL_DRIVE

if __name__ == '__main__':
    wpilib.run(Robot)