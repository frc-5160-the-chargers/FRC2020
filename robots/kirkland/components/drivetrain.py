from magicbot import tunable, StateMachine, state

from wpilib import SpeedControllerGroup, PIDController
from wpilib.drive import DifferentialDrive

from ctre import WPI_TalonSRX
from navx import AHRS

from robotmap import RobotMap
from dash import get_pid, put_pid
from components.navx_component import NavX
from kinematics import ArcDrive
from pid import SuperPIDController, ff_constant, ff_flywheel

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
        return i / RobotMap.Encoders.ticks_per_rotation * circ / RobotMap.Encoders.output_gear_ratio

    def convert_velocity_meters_per_second(self, i):
        circ = math.pi * 2 * RobotMap.Encoders.wheel_diameter
        return i * circ / RobotMap.Encoders.output_gear_ratio / RobotMap.Encoders.ticks_per_rotation / RobotMap.Encoders.velocity_period 

    def get_position(self, side=EncoderSide.AVERAGE):
        assert (side in [EncoderSide.LEFT, EncoderSide.RIGHT, EncoderSide.AVERAGE]), "Invalid encoder side"
        if side == EncoderSide.LEFT:
            return -self.convert_ticks_meter(self.encoder_left.getQuadraturePosition())
        elif side == EncoderSide.RIGHT:
            return self.convert_ticks_meter(self.encoder_right.getQuadraturePosition())
        elif side == EncoderSide.AVERAGE:
            return self.convert_ticks_meter(
                (-self.encoder_left.getQuadraturePosition() + self.encoder_right.getQuadraturePosition()) / 2
            )
            
    def get_velocity(self, side=EncoderSide.AVERAGE):
        assert (side in [EncoderSide.LEFT, EncoderSide.RIGHT, EncoderSide.AVERAGE]), "Invalid encoder side"
        if side == EncoderSide.LEFT:
            return -self.convert_velocity_meters_per_second(self.encoder_left.getQuadratureVelocity())
        elif side == EncoderSide.RIGHT:
            return self.convert_velocity_meters_per_second(self.encoder_right.getQuadratureVelocity())
        elif side == EncoderSide.AVERAGE:
            return self.convert_velocity_meters_per_second(
                (-self.encoder_left.getQuadratureVelocity() + self.encoder_right.getQuadratureVelocity()) / 2
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

    def set_power_left(self, power):
        self.left_power = power

        self.current_state = PowertrainMode.TANK

        self.power = 0
        self.rotation = 0

    def set_power_right(self, power):
        self.right_power = power

        self.current_state = PowertrainMode.TANK

        self.power = 0
        self.rotation = 0

    def reset(self):
        self.power = 0
        self.rotation = 0

        self.left_power = 0
        self.right_power = 0

        self.current_state = PowertrainMode.CURVATURE

    def execute(self):
        if self.current_state == PowertrainMode.CURVATURE:
            self.drivetrain.curvatureDrive(self.power, self.rotation, True)
        
        if self.current_state == PowertrainMode.TANK:
            self.drivetrain.tankDrive(self.left_power, self.right_power, False)

        if self.current_state == PowertrainMode.ARCADE:
            self.drivetrain.arcadeDrive(self.power, self.rotation, False)
    
class DriveMode:
    MANUAL_DRIVE = 0
    AIDED_TURN = 1
    PID_TURN = 2
    PID_DRIVE = 3
    VELOCITY_CONTROL = 4

class Drivetrain:
    powertrain: Powertrain
    encoders: Encoders

    navx_component: NavX
    
    def __init__(self):
        self.turn_pid = SuperPIDController(
            pid_values=RobotMap.Drivetrain.turn_pid,
            f_in=self.get_heading,
            f_out=lambda x: self.powertrain.set_arcade_turning_speed(x),
            f_feedforwards=lambda target, error: ff_constant(RobotMap.Drivetrain.kF_turn, target, error),
            pid_key=RobotMap.Drivetrain.turn_pid_key 
        )
        self.turn_pid.configure_controller(
            output_range=(-1, 1),
            percent_tolerance=1
        )

        self.position_pid = SuperPIDController(
            pid_values=RobotMap.Drivetrain.position_pid,
            f_in=self.get_position,
            f_out=lambda x: self.powertrain.set_arcade_power(x),
            f_feedforwards=lambda target, error: ff_constant(RobotMap.Drivetrain.kF_straight, target, error),
            pid_key=RobotMap.Drivetrain.position_pid_key
        )
        self.position_pid.configure_controller(
            output_range=(-1, 1),
            percent_tolerance=1
        )

        self.velocity_left_pid = SuperPIDController(
            pid_values=RobotMap.Drivetrain.velocity_left,
            f_in=lambda: self.get_velocity(EncoderSide.LEFT),
            f_out=lambda x: self.powertrain.set_power_left(x),
            f_feedforwards=lambda target, error: ff_flywheel(RobotMap.Drivetrain.kF_velocity, target, error),
            pid_key=RobotMap.Drivetrain.velocity_left_key
        )
        self.velocity_left_pid.configure_controller(
            output_range=(-1, 1),
            percent_tolerance=1
        )

        self.velocity_right_pid = SuperPIDController(
            pid_values=RobotMap.Drivetrain.velocity_right,
            f_in=lambda: self.get_velocity(EncoderSide.RIGHT),
            f_out=lambda x: self.powertrain.set_power_right(x),
            f_feedforwards=lambda target, error: ff_flywheel(RobotMap.Drivetrain.kF_velocity, target, error),
            pid_key=RobotMap.Drivetrain.velocity_right_key
        )
        self.velocity_right_pid.configure_controller(
            output_range=(-1, 1),
            percent_tolerance=1
        )

        self.pid_controllers = [
            self.turn_pid,
            self.position_pid,
            self.velocity_left_pid,
            self.velocity_right_pid
        ]

        self.drive_mode = DriveMode.MANUAL_DRIVE

    def get_velocity(self, side: EncoderSide):
        return self.encoders.get_velocity(side)

    def update_pid_dash(self):
        for controller in self.pid_controllers:
            controller.update_from_dash()

    def push_pid_dash(self):
        for controller in self.pid_controllers:
            controller.push_to_dash()

    def reset(self):
        for controller in self.pid_controllers:
            controller.reset()

        self.powertrain.reset()
        self.encoders.reset()

        self.navx_component.reset()

        self.drive_mode = DriveMode.MANUAL_DRIVE

    def get_heading(self):
        return self.navx_component.get_angle()

    def get_position(self):
        return self.encoders.get_position()

    def get_pid_on_point(self):
        return False not in set([
            controller.get_on_target() for controller in self.pid_controllers
        ])

    def stop_pid_controllers(self):
        for controller in self.pid_controllers:
            controller.stop()

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
            self.navx_component.reset()
            self.drive_mode = DriveMode.PID_TURN
            self.turn_pid.run_setpoint(angle)

    def drive_to_position(self, position):
        if self.drive_mode != DriveMode.PID_DRIVE:
            self.stop_pid_controllers()
            self.encoders.reset()
            self.drive_mode = DriveMode.PID_DRIVE
            self.position_pid.run_setpoint(position)

    def velocity_control(self, velocity_left, velocity_right):
        if self.drive_mode != DriveMode.VELOCITY_CONTROL:
            self.stop_pid_controllers()
            self.encoders.reset()
            self.drive_mode = DriveMode.VELOCITY_CONTROL
            self.velocity_left_pid.run_setpoint(velocity_left)
            self.velocity_right_pid.run_setpoint(velocity_right)

    def arc_drive(self, constraints: ArcDrive):
        if constraints.valid:
            self.velocity_control(constraints.v_l, constraints.v_r)
        else:
            self.powertrain.drive_tank(0, 0)

    def execute(self):
        pass