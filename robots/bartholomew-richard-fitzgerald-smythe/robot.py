import wpilib
import wpilib.drive

from wpilib import SmartDashboard as dash

import magicbot

from rev import CANSparkMax, MotorType
from rev.color import ColorSensorV3
from ctre import WPI_TalonSRX

import navx

from oi import Driver, Sysop
from utils import config_spark, config_talon
from robotmap import RobotMap
from dash import Tunable

from components.drivetrain import Drivetrain, Powertrain, DrivetrainState, EncoderSide
from components.intake import IntakeLift, IntakeRoller, Intake, IntakeLiftState
from components.climber import Climber
from components.sensors import Encoders, NavX

class Robot(magicbot.MagicRobot):
    powertrain: Powertrain
    encoders: Encoders
    navx: NavX

    drivetrain: Drivetrain

    intake_lift: IntakeLift
    intake_roller: IntakeRoller

    intake: Intake

    climber: Climber

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

        # climber
        self.climber_motor = WPI_TalonSRX(RobotMap.Climber.motor_port)
        config_talon(self.climber_motor, RobotMap.Climber.motor_config)

        # color wheel
        self.color_wheel_motor = WPI_TalonSRX(RobotMap.ColorWheel.motor_port)
        config_talon(self.color_wheel_motor, RobotMap.ColorWheel.motor_config)

        self.color_sensor = ColorSensorV3()

        # controllers and electrical stuff
        self.driver = Driver(wpilib.XboxController(0))
        self.sysop = Sysop(wpilib.XboxController(1))

        # camera server
        wpilib.CameraServer.launch()

    def reset_subsystems(self):
        self.drivetrain.reset()
        self.intake.reset() # TODO this'll need to be different when we run auto
        self.climber.reset()

    def teleopInit(self):
        self.reset_subsystems()

    def teleopPeriodic(self):
        try:
            # drive the drivetrain as needed
            driver_x, driver_y = self.driver.get_curvature_output()

            # manually handled driving
            if self.drivetrain.state == DrivetrainState.MANUAL_DRIVE:
                self.drivetrain.curvature_drive(driver_y, driver_x)

            # # check and see if we need to activate driver assists
            # if self.drivetrain.ready_straight_assist() and self.driver.ready_straight_assist():
            #     self.drivetrain.drive_straight(driver_y)
            # elif self.drivetrain.state == DrivetrainState.AIDED_DRIVE_STRAIGHT:
            #     self.drivetrain.state = DrivetrainState.MANUAL_DRIVE

            # # revert to manual control if enabled
            # if self.driver.get_manual_control_override():
            #     self.drivetrain.state = DrivetrainState.MANUAL_DRIVE
        except:
            print("DRIVETRAIN ERROR")

        try:
            # move intake lift
            if self.sysop.get_intake_lower():
                self.intake_lift.send_down()
            elif self.sysop.get_intake_raise():
                self.intake_lift.send_up()
        except:
            print("INTAKE LIFT ERROR")

        try:
            # intake rollers
            if self.sysop.get_intake_intake():
                self.intake_roller.intake()
            elif self.sysop.get_intake_outtake():
                self.intake_roller.outtake()
            else:
                self.intake_roller.stop()
        except:
            print("INTAKE ROLLER ERROR")

        try:
            self.climber.set_power(self.sysop.get_climb_axis())
        except:
            print("CLIMBER ERROR")

if __name__ == '__main__':
    git_gud = lambda: wpilib.run(Robot)
    git_gud()