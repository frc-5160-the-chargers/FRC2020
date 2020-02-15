from rev import CANSparkMax

from wpilib.drive import DifferentialDrive
from wpilib import SpeedControllerGroup

from robotmap import RobotMap

from components.sensors import NavX, Encoders, EncoderSide

from pid import SuperPIDController, ff_constant, ff_flywheel, PidManager
from kinematics import ArcDrive

class PowertrainMode:
    TANK_DRIVE = 0
    CURVATURE_DRIVE = 1
    ARCADE_DRIVE = 2

class Powertrain:
    left_motors: SpeedControllerGroup
    right_motors: SpeedControllerGroup

    differential_drive: DifferentialDrive

    def __init__(self):
        self.reset_state()

    def reset_state(self):
        self.mode = PowertrainMode.CURVATURE_DRIVE

        self.power = 0
        self.rotation = 0
        self.left_power = 0
        self.right_power = 0 
    
    def reset(self):
        self.reset_state()

    # oh god, what atrocities have i committed.

    def set_tank_powers(self, left_power, right_power):
        self.power = self.rotation = 0
        self.left_power = left_power
        self.right_power = right_power

    def set_arcade_powers(self, power, rotation):
        self.left_power = self.right_power = 0 
        self.power = power
        self.rotation = rotation

    def set_arcade_turning(self, rotation):
        self.mode = PowertrainMode.ARCADE_DRIVE
        self.left_power = self.right_power = 0
        self.rotation = rotation
    
    def set_arcade_speed(self, power):
        self.mode = PowertrainMode.ARCADE_DRIVE
        self.left_power = self.right_power = 0
        self.power = power

    def set_power_left(self, power):
        self.mode = PowertrainMode.TANK_DRIVE
        self.power = self.rotation = 0
        self.left_power = power

    def set_power_right(self, power):
        self.mode = PowertrainMode.TANK_DRIVE
        self.power = self.rotation = 0
        self.right_power = power

    def tank_drive(self, left_power, right_power):
        self.mode = PowertrainMode.TANK_DRIVE
        self.set_tank_powers(left_power, right_power)

    def arcade_drive(self, power, rotation):
        self.mode = PowertrainMode.ARCADE_DRIVE
        self.set_arcade_powers(power, rotation)

    def curvature_drive(self, power, rotation):
        self.mode = PowertrainMode.CURVATURE_DRIVE
        self.set_arcade_powers(power, rotation)

    def execute(self):
        if self.mode == PowertrainMode.TANK_DRIVE:
            self.differential_drive.tankDrive(self.left_power, self.right_power, False)
        elif self.mode == PowertrainMode.ARCADE_DRIVE:
            self.differential_drive.arcadeDrive(self.power, self.rotation, False)
        elif self.mode == PowertrainMode.CURVATURE_DRIVE:
            self.differential_drive.curvatureDrive(self.power, self.rotation, False)

class DrivetrainState:
    MANUAL_DRIVE = 0
    
    AIDED_DRIVE = 10
    AIDED_DRIVE_STRAIGHT_TAKEOVER = 11

    PID_TURNING = 20
    PID_STRAIGHT = 21
    PID_VELOCITY = 22

class Drivetrain:
    powertrain: Powertrain
    encoders: Encoders
    navx: NavX

    def __init__(self):
        self.turn_pid = SuperPIDController(
            pid_values=RobotMap.Drivetrain.turn_pid,
            f_in=lambda: self.get_heading(),
            f_out=lambda x: self.powertrain.set_arcade_turning(x),
            f_feedforwards=lambda target, error: ff_constant(RobotMap.Drivetrain.kF_turn, target, error),
            pid_key=RobotMap.Drivetrain.turn_pid_key
        )
        self.turn_pid.configure_controller(
            output_range=(-1, 1),
            percent_tolerance=1
        )

        self.position_pid = SuperPIDController(
            pid_values=RobotMap.Drivetrain.position_pid,
            f_in=lambda: self.get_position(EncoderSide.BOTH),
            f_out=lambda x: self.powertrain.set_arcade_speed(x),
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

        self.pid_manager = PidManager([
            self.turn_pid,
            self.position_pid,
            self.velocity_left_pid,
            self.velocity_right_pid
        ])

    def reset_state(self):
        self.state = DrivetrainState.MANUAL_DRIVE

    def reset(self):
        self.pid_manager.reset_controllers()

        self.powertrain.reset()
        self.encoders.reset()
        self.navx.reset()

        self.reset_state()

    def get_heading(self):
        return self.navx.get_heading()

    def get_velocity(self, side: EncoderSide):
        return self.encoders.get_velocity(side)

    def get_position(self, side: EncoderSide):
        return self.encoders.get_position(side)

    def tank_drive(self, left_power, right_power):
        self.pid_manager.stop_controllers()
        self.state = DrivetrainState.MANUAL_DRIVE
        self.powertrain.tank_drive(left_power, right_power)

    def curvature_drive(self, power, rotation):
        self.pid_manager.stop_controllers()
        self.state = DrivetrainState.MANUAL_DRIVE
        self.powertrain.curvature_drive(power, rotation)

    def turn_to_angle(self, angle):
        if self.state != DrivetrainState.PID_TURNING:
            self.pid_manager.stop_controllers()
            self.navx.reset()
            self.state = DrivetrainState.PID_TURNING
            self.turn_pid.run_setpoint(angle)

    def drive_to_position(self, position):
        if self.state != DrivetrainState.PID_STRAIGHT:
            self.pid_manager.stop_controllers()
            self.encoders.reset()
            self.state = DrivetrainState.PID_STRAIGHT
            self.position_pid.run_setpoint(position)

    def velocity_control(self, left_velocity, right_velocity):
        if self.state != DrivetrainState.PID_VELOCITY:
            self.pid_manager.stop_controllers()
            self.encoders.reset()
            self.state = DrivetrainState.PID_VELOCITY
            self.velocity_left_pid.run_setpoint(left_velocity)
            self.velocity_right_pid.run_setpoint(right_velocity)

    # TODO fully implement arc drive as defined in kinematics
    # once that's done we can then do bezier curve nonsense
    def arc_drive(self, contraints: ArcDrive):
        if contraints.valid:
            self.velocity_control(contraints.v_l, contraints.v_r)
        else:
            self.powertrain.tank_drive(0, 0)

    def execute(self):
        pass