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

    # refactorsed and genericisized setters for power (good to use in PID controllers)
    def set_tank_powers(self, left_power=None, right_power=None):
        self.power = self.rotation = 0
        if left_power != None:
            self.left_power = left_power
        if right_power != None:
            self.right_power = right_power

    def set_arcade_powers(self, power=None, rotation=None):
        self.left_power = self.right_power = 0 
        if power != None:
            self.power = power
        if rotation != None:
            self.rotation = rotation

    # basic drive setters
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
            self.differential_drive.curvatureDrive(self.power, self.rotation, True)

class DrivetrainState:
    # 0-9 == manual modes
    MANUAL_DRIVE = 0
    
    # 10-19 == aided modes
    AIDED_DRIVE_STRAIGHT = 10

    # 20-29 == PID modes
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
            f_out=lambda x: self.powertrain.set_arcade_powers(rotation=x),
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
            f_out=lambda x: self.powertrain.set_arcade_powers(power=x),
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
            f_out=lambda x: self.powertrain.set_tank_powers(left_power=x),
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
            f_out=lambda x: self.powertrain.set_tank_powers(right_power=x),
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

        self.reset_state()

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

    def ready_straight_assist(self):
        return self.navx.navx_ahrs.isConnected()

    def tank_drive(self, left_power, right_power):
        self.pid_manager.stop_controllers()
        self.state = DrivetrainState.MANUAL_DRIVE
        self.powertrain.tank_drive(left_power, right_power)

    def curvature_drive(self, power, rotation):
        self.pid_manager.stop_controllers()
        self.state = DrivetrainState.MANUAL_DRIVE
        self.powertrain.curvature_drive(power, rotation)

    def drive_straight(self, power):
        if self.state != DrivetrainState.AIDED_DRIVE_STRAIGHT:
            self.pid_manager.stop_controllers()
            self.navx.reset()
            self.state = DrivetrainState.AIDED_DRIVE_STRAIGHT
            self.powertrain.mode = PowertrainMode.ARCADE_DRIVE
            self.turn_pid.run_setpoint(0)
        self.powertrain.set_arcade_powers(power=power)

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