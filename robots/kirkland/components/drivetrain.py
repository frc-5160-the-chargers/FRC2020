from magicbot import tunable

from wpilib import SpeedControllerGroup
from wpilib.drive import DifferentialDrive

from ctre import WPI_TalonSRX

from navx import AHRS

from config import load_config_file

from components.navx import NavX

import math

config_data = load_config_file()

class Encoders:
    encoder_left: WPI_TalonSRX
    encoder_right: WPI_TalonSRX

    def __init__(self):
        pass

    def convert_ticks_meter(self, i):
        ticks_rotation = config_data['constants']['ticks_per_rotation']
        wheel_diam = config_data['constants']['wheel_diameter']
        circ = math.pi * 2 * wheel_diam
        return i / ticks_rotation * circ

    def get_position(self, side="average"):
        assert (side in ["left", "right", "average"]), "Invalid encoder side"
        if side == "left":
            return self.convert_ticks_meter(self.encoder_left.getQuadraturePosition())
        elif side == "right":
            return self.convert_ticks_meter(self.encoder_right.getQuadraturePosition())
        elif side == "average":
            return self.convert_ticks_meter(
                (self.encoder_left.getQuadraturePosition() + self.encoder_right.getQuadraturePosition()) / 2
            )
    
    def get_velocity(self, side="average"):
        assert (side in ["left", "right", "average"]), "Invalid encoder side"
        if side == "left":
            return self.convert_ticks_meter(self.encoder_left.getQuadratureVelocity())
        elif side == "right":
            return self.convert_ticks_meter(self.encoder_right.getQuadratureVelocity())
        elif side == "average":
            return self.convert_ticks_meter(
                (self.encoder_left.getQuadratureVelocity() + self.encoder_right.getQuadratureVelocity()) / 2
            )

    def reset(self):
        self.reset_encoder_positions()

    def reset_encoder_positions(self):
        self.encoder_left.setQuadraturePosition(0)
        self.encoder_right.setQuadraturePosition(0)

    def execute(self):
        pass

class Powertrain:
    motors_left: SpeedControllerGroup
    motors_right: SpeedControllerGroup
    drivetrain: DifferentialDrive

    def __init__(self):
        self.power = 0
        self.rotation = 0

    def configure_motors(self, motor_list=[]):
        if motor_list == []:
            self.configure_motors(motors_list=self.motors_left.speedControllers)
            self.configure_motors(motors_list=self.motors_right.speedControllers)
            return
        for motor in motor_list:
            assert type(motor) == WPI_TalonSRX, "Motor controller is not TalonSRX"
            motor: WPI_TalonSRX
            
            motor.enableVoltageCompensation(True)
            motor.configVoltageCompSaturation(config_data['motor_config']['drivetrain']['voltage_saturation'])
            
            motor.enableCurrentLimit(True)
            motor.configPeakCurrentLimit(config_data['motor_config']['drivetrain']['peak_current'])
            motor.configContinuousCurrentLimit(config_data['motor_config']['drivetrain']['continuous_current'])

            motor.controlMode = motor.ControlMode.PercentOutput

            default_mode = config_data['motor_config']['drivetrain']['default_mode']
            assert (default_mode in ['break', 'coast']), 'Drivetrain defalt mode not valid'
            motor.setNeutralMode({
                'break': motor.NeutralMode.Brake,
                'coast': motor.NeutralMode.Coast
            }[default_mode])

            motor.configNeutralDeadband(config_data['motor_config']['drivetrain']['deadband'])
        
    def drive(self, power, rotation):
        self.power = power
        self.rotation = rotation

    def reset(self):
        self.power = 0
        self.rotation = 0

    def execute(self):
        self.drivetrain.curvatureDrive(self.power, self.rotation, False)
    
class Drivetrain:
    powertrain: Powertrain
    encoders: Encoders

    navx_component: NavX

    DRIVE_STRAIGHT = 0
    DRIVE_TURN = 1

    turn_hold_kP = tunable(default=0)

    def __init__(self):
        self.rotation_deadband = config_data['subsystem_config']['drivetrain']['rotation_deadzone']

        self.drive_mode = Drivetrain.DRIVE_STRAIGHT
        self.angle_hold = 0

    def reset(self):
        self.powertrain.reset()
        self.encoders.reset()

        self.drive_mode = Drivetrain.DRIVE_STRAIGHT
        self.angle_hold = 0

        self.navx_component.reset()

    def drive(self, power, rotation):
        if self.drive_mode == Drivetrain.DRIVE_STRAIGHT and abs(rotation) > self.rotation_deadband:
            self.drive_mode = Drivetrain.DRIVE_TURN
        if self.drive_mode == Drivetrain.DRIVE_TURN and abs(rotation) < self.rotation_deadband:
            self.drive_mode = Drivetrain.DRIVE_STRAIGHT
            self.angle_hold = self.navx_component.get_angle()

        if self.drive_mode == Drivetrain.DRIVE_STRAIGHT:
            angle_difference = self.navx_component.get_angle() - self.angle_hold
            self.powertrain.drive(power, angle_difference*self.turn_hold_kP)
        elif self.drive_mode == Drivetrain.DRIVE_TURN:
            self.powertrain.drive(power, rotation)

    def execute(self):
        pass
