from magicbot import tunable, StateMachine, state

from wpilib import SpeedControllerGroup, PIDController
from wpilib.drive import DifferentialDrive

from ctre import WPI_TalonSRX
from navx import AHRS

from robotmap import RobotMap
from dash import get_pid, push_pid
from components.navx import NavX

import math

class EncoderSide:
    LEFT = 0
    RIGHT = 1
    AVERAGE = 2

class Encoders:
    encoder_left: WPI_TalonSRX
    encoder_right: WPI_TalonSRX

    def __init__(self):
        pass

    def convert_ticks_meter(self, i):
        circ = math.pi * 2 * RobotMap.Encoders.wheel_diameter
        return i / RobotMap.Encoders.ticks_per_rotation * circ

    def get_position(self, side=EncoderSide.AVERAGE):
        assert (side in [Encoders.Side.LEFT, EncoderSide.RIGHT, EncoderSide.AVERAGE]), "Invalid encoder side"
        if side == EncoderSide.LEFT:
            return self.convert_ticks_meter(self.encoder_left.getQuadraturePosition())
        elif side == EncoderSide.RIGHT:
            return self.convert_ticks_meter(self.encoder_right.getQuadraturePosition())
        elif side == EncoderSide.AVERAGE:
            return self.convert_ticks_meter(
                (self.encoder_left.getQuadraturePosition() + self.encoder_right.getQuadraturePosition()) / 2
            )
            
    def get_velocity(self, side=EncoderSide.AVERAGE):
        assert (side in [EncoderSide.LEFT, EncoderSide.RIGHT, EncoderSide.AVERAGE]), "Invalid encoder side"
        if side == EncoderSide.LEFT:
            return self.convert_ticks_meter(self.encoder_left.getQuadratureVelocity())
        elif side == EncoderSide.RIGHT:
            return self.convert_ticks_meter(self.encoder_right.getQuadratureVelocity())
        elif side == EncoderSide.AVERAGE:
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

class PowertrainMode:
    CURVATURE = 0
    TANK = 1
    ARCADE = 2

class Powertrain:
    motors_left: SpeedControllerGroup
    motors_right: SpeedControllerGroup
    drivetrain: DifferentialDrive

    def __init__(self):
        self.power = 0
        self.rotation = 0

        self.left_power = 0
        self.right_power = 0

        self.current_state = PowertrainMode.CURVATURE

    def drive_curvature(self, power, rotation):
        self.power = power
        self.rotation = rotation

        self.current_state = PowertrainMode.CURVATURE

        self.left_power = 0
        self.right_power = 0
    
    def drive_tank(self, left_power, right_power):
        self.left_power = left_power
        self.right_power = right_power

        self.current_state = PowertrainMode.TANK

        self.power = 0
        self.rotation = 0

    def drive_arcade(self, power, rotation):
        self.power = power
        self.rotation = rotation

        self.current_state = PowertrainMode.ARCADE

        self.left_power = 0
        self.right_power = 0

    def set_arcade_turning_speed(self, rotation):
        self.rotation = rotation

        self.current_state = PowertrainMode.ARCADE

        self.left_power = 0
        self.right_power = 0

    def set_arcade_power(self, power):
        self.power = power
        
        self.current_state = PowertrainMode.ARCADE

        self.left_power = 0
        self.right_power = 0

    def reset(self):
        self.power = 0
        self.rotation = 0

        self.left_power = 0
        self.right_power = 0

        self.current_state = PowertrainMode.CURVATURE

    def execute(self):
        if self.current_state == PowertrainMode.CURVATURE:
            self.drivetrain.curvatureDrive(self.power, self.rotation, False)
        
        if self.current_state == PowertrainMode.TANK:
            self.drivetrain.tankDrive(self.left_power, self.right_power, False)

        if self.current_state == PowertrainMode.ARCADE:
            self.drivetrain.arcadeDrive(self.power, self.rotation, False)
    
class DriveMode:
    MANUAL_DRIVE = 0
    AIDED_TURN = 1
    PID_TURN = 2
    PID_DRIVE = 3

class Drivetrain:
    powertrain: Powertrain
    encoders: Encoders

    navx_component: NavX
    
    def __init__(self):
        self.turn_pid_values = RobotMap.Drivetrain.turn_pid
        self.position_pid_values = RobotMap.Drivetrain.position_pid

        self.turn_pid = PIDController(0, 0, 0, self.get_heading, lambda x: self.powertrain.set_arcade_turning_speed(x))
        self.turn_pid_values.update_controller(self.turn_pid)
        self.turn_pid.setOutputRange(-1, 1)
        self.turn_pid.setPercentTolerance(5)

        self.position_pid = PIDController(0, 0, 0, self.get_position, lambda x: self.powertrain.set_arcade_power(x))
        self.position_pid_values.update_controller(self.position_pid)
        self.position_pid.setOutputRange(-1, 1)
        self.position_pid.setPercentTolerance(5)

        self.drive_mode = DriveMode.MANUAL_DRIVE

    def update_pid_dash(self):
        self.turn_pid_values = get_pid(RobotMap.Drivetrain.turn_pid_key)
        self.position_pid_values = get_pid(RobotMap.Drivetrain.position_pid_key)

        self.turn_pid_values.update_controller(self.turn_pid)
        self.position_pid_values.update_controller(self.position_pid)

    def push_pid_dash(self):
        push_pid(RobotMap.Drivetrain.turn_pid_key, self.turn_pid_values)
        push_pid(RobotMap.Drivetrain.position_pid_key, self.position_pid_values)

    def reset(self):
        self.turn_pid.reset()
        self.position_pid.reset()

        self.powertrain.reset()
        self.encoders.reset()

        self.navx_component.reset()

        self.drive_mode = DriveMode.MANUAL_DRIVE

    def get_heading(self):
        return self.navx_component.get_angle()

    def get_position(self):
        return self.encoders.get_position()

    def get_pid_on_point(self):
        return (
            (self.turn_pid.onTarget() if self.turn_pid.isEnabled() else True) and
            (self.position_pid.onTarget() if self.position_pid.isEnabled() else True)
        )

    def stop_pid_controllers(self):
        self.turn_pid.disable()
        self.position_pid.disable()

    def tank_drive(self, left_power, right_power):
        self.stop_pid_controllers()
        self.drive_mode = DriveMode.MANUAL_DRIVE
        self.powertrain.drive_tank(left_power, right_power)

    def curvature_drive(self, power, rotation):
        self.stop_pid_controllers()
        self.drive_mode = DriveMode.MANUAL_DRIVE
        self.powertrain.drive_curvature(power, rotation)

    def turn_to_angle(self, angle):
        if self.drive_mode != DriveMode.PID_TURN:
            self.stop_pid_controllers()
            self.drive_mode = DriveMode.PID_TURN
            self.turn_pid.setSetpoint(angle)
            self.turn_pid.enable()

    def drive_to_position(self, position):
        if self.drive_mode != DriveMode.PID_DRIVE:
            self.stop_pid_controllers()
            self.drive_mode = DriveMode.PID_DRIVE
            self.position_pid.setSetpoint(position)
            self.position_pid.enable()

    def execute(self):
        pass