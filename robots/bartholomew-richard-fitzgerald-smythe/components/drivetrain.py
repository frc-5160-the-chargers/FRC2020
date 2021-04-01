from rev import CANSparkMax

from wpilib.drive import DifferentialDrive
from wpilib import SpeedControllerGroup

from robotmap import RobotMap

from components.position_approximation import PosApprox
from components.sensors import NavX, Encoders, EncoderSide

from pid import SuperPIDController, ff_constant, ff_flywheel, PidManager
from kinematics import ArcDrive

from components.limelight import Limelight
from components.shooter import Shooter
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
        self.mode = PowertrainMode.TANK_DRIVE
        self.power = self.rotation = 0
        if left_power != None:
            self.left_power = left_power
        if right_power != None:
            self.right_power = right_power

    def set_arcade_powers(self, power=None, rotation=None):
        self.mode = PowertrainMode.ARCADE_DRIVE
        self.left_power = self.right_power = 0 
        
        if power != None:
            #print(f"power set: {power}");
            self.power = power
        if rotation != None:
            #print("rotation set",rotation )
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
    STOPPPED = 11
    # 20-29 == PID modes
    PID_TURNING = 20
    PID_STRAIGHT = 21
    PID_LIMELIGHT_TURNING = 22;
    PID_LIMELIGHT_DRIVE = 23;
    
class Drivetrain:
    location: PosApprox
    powertrain: Powertrain
    encoders: Encoders
    navx: NavX
    limelight : Limelight
    #shooter : Shooter

    def __init__(self):
        self.turn_pid = SuperPIDController(
            pid_values=RobotMap.Drivetrain.turn_pid,
            f_in=lambda: self.get_heading(),
            f_out=lambda x: self.powertrain.set_arcade_powers(rotation=x),
            f_feedforwards=lambda target, error: ff_constant(RobotMap.Drivetrain.kF_turn, target, error),
            pid_key=RobotMap.Drivetrain.turn_pid_key
        )
        self.turn_pid.configure_controller(
            output_range=(-RobotMap.Drivetrain.max_auto_power, RobotMap.Drivetrain.max_auto_power),
            tolerance=1
        )

        self.position_pid = SuperPIDController(
            pid_values=RobotMap.Drivetrain.position_pid,
            f_in=lambda: self.get_position(EncoderSide.BOTH),
            f_out=lambda x: self.powertrain.set_arcade_powers(power=x),
            f_feedforwards=lambda target, error: ff_constant(RobotMap.Drivetrain.kF_straight, target, error),
            pid_key=RobotMap.Drivetrain.position_pid_key
        )
        self.position_pid.configure_controller(
            output_range=(-RobotMap.Drivetrain.max_auto_power, RobotMap.Drivetrain.max_auto_power),
            tolerance=0.5
        )

        self.limelight_turn_pid = SuperPIDController(
            pid_values = RobotMap.Drivetrain.limelight_turn_pid,
            f_in=lambda: self.limelight.get_last_horizontal_angle_offset(),
            f_out=lambda x: self.powertrain.set_arcade_powers(rotation=x),
            f_feedforwards=lambda target, error: ff_constant(0.2,target,error),
            pid_key=RobotMap.Drivetrain.limelight_turn_pid_key           
        )
        self.limelight_turn_pid.configure_controller(
            output_range=(-RobotMap.Drivetrain.max_auto_power, RobotMap.Drivetrain.max_auto_power),
            tolerance=0.5 
        )

        self.limelight_distance_pid = SuperPIDController(
            pid_values = RobotMap.Drivetrain.limelight_distance_pid,
            f_in=lambda: self.limelight.get_last_distance_trig(0.12), #TODO: MAKE PARTIAL FUNCTION OR PASS KWARGS OR **SOMETHING**, NOT THIS
            f_out=lambda x: self.powertrain.set_arcade_powers(power=x),
            f_feedforwards=lambda target, error: ff_constant(RobotMap.Drivetrain.kF_straight, target, error),
            pid_key=RobotMap.Drivetrain.limelight_distance_pid_key
        )
        self.limelight_distance_pid.configure_controller(
            output_range=(-RobotMap.Drivetrain.max_auto_power, RobotMap.Drivetrain.max_auto_power),
            tolerance=1 #tune
        )

        self.pid_manager = PidManager([
            self.turn_pid,
            self.position_pid,
            self.limelight_turn_pid,
            self.limelight_distance_pid,
        ])

        


        self.reset_state()

    def reset_state(self):
        self.state = DrivetrainState.MANUAL_DRIVE
        self.callback = None

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
            self.state = DrivetrainState.AIDED_DRIVE_STRAIGHT
            self.powertrain.mode = PowertrainMode.ARCADE_DRIVE
            self.turn_pid.run_setpoint(self.navx.get_heading())
        self.powertrain.set_arcade_powers(power=power)
    
    def aim_at_target(self):
        if self.state == DrivetrainState.MANUAL_DRIVE:
            self.turn_to_angle(self.limelight.get_horizontal_angle_offset(), self.start_fire)

    def start_fire(self):
        self.shooter.fire()
        self.state = DrivetrainState.STOPPPED

    def turn_to_limelight_target(self,next = None):
        if self.state != DrivetrainState.PID_LIMELIGHT_TURNING:
            self.pid_manager.stop_controllers();
            self.powertrain.reset_state();
            self.state = DrivetrainState.PID_LIMELIGHT_TURNING;
            self.limelight_turn_pid.run_setpoint(0);
            self.callback = next;
            
    def turn_to_angle(self, angle, next = None, global=False):
        if self.state != DrivetrainState.PID_TURNING or (angle + (self.navx.get_heading() if not global else 0)) != self.turn_pid.get_target():
            self.pid_manager.stop_controllers()
            self.powertrain.reset_state();
            self.state = DrivetrainState.PID_TURNING
            self.turn_pid.run_setpoint(angle + (self.navx.get_heading() if not global else 0))
            self.callback = next

    def drive_to_position(self, position):
        if self.state != DrivetrainState.PID_STRAIGHT:
            self.pid_manager.stop_controllers()
            self.powertrain.reset_state();
            self.state = DrivetrainState.PID_STRAIGHT
            self.position_pid.run_setpoint(position + self.encoders.get_position(EncoderSide.BOTH));

    def drive_to_limelight_target(self,distance,target_height):
        if self.state != DrivetrainState.PID_LIMELIGHT_DRIVE:
            self.pid_manager.stop_controllers();
            self.powertrain.reset_state();
            self.state = DrivetrainState.PID_LIMELIGHT_DRIVE;
            self.limelight_turn_pid.run_setpoint(0);
            self.limelight_distance_pid.run_setpoint(distance);

    def set_power_scaling(self, new_power_scaling):
        self.powertrain.differential_drive.setMaxOutput(new_power_scaling)

    def stop(self):
        self.tank_drive(0, 0)

    def execute(self):
        self.pid_manager.execute_controllers()
        if ((self.state == DrivetrainState.PID_TURNING and self.turn_pid.get_on_target())
            or (self.state == DrivetrainState.PID_LIMELIGHT_TURNING and self.limelight_turn_pid.get_on_target())):
            if self.callback is not None:
                self.callback();
