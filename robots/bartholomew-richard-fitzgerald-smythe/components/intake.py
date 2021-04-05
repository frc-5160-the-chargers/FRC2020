from wpilib import Encoder
from ctre import WPI_TalonSRX

from robotmap import RobotMap

from utils import clamp
from pid import SuperPIDController, PidManager

class IntakeLiftState:
    STOPPED = 0
    RAISING = 1
    LOWERING = 2

    PID_CONTROLLED = 10

class IntakeLiftPosition:
    MATCH_START = 0
    LOWERED = 1
    RAISED = 2

class IntakeLift:
    intake_lift_motor: WPI_TalonSRX
    # intake_lift_encoder: Encoder

    def __init__(self):
        self.pid_controller = SuperPIDController(
            pid_values=RobotMap.IntakeLift.pid_values,
            f_in=self.get_position,
            f_out=lambda x: self.set_power_raw(x),
            f_feedforwards=lambda a, b: self.get_feedforwards(a, b),
            pid_key=RobotMap.IntakeLift.pid_key
        )
        self.pid_controller.configure_controller(
            output_range=(-RobotMap.IntakeLift.max_power_down, RobotMap.IntakeLift.max_power_up),
            tolerance=1
        )
        self.reset_state()

    def reset_state(self):
        self.state = IntakeLiftState.STOPPED
        self.power = 0
        self.position = IntakeLiftPosition.MATCH_START
        # self.target_position = 0

    def reset_encoder(self):
        self.intake_lift_motor.setSelectedSensorPosition(0)

    def reset(self):
        self.reset_state()
        self.reset_encoder()
        # self.intake_lift_encoder.reset()

    def get_position(self):
        # return self.intake_lift_encoder.getDistance()
        return self.intake_lift_motor.getSelectedSensorPosition() * RobotMap.IntakeLift.encoder_distance_per_pulse

    def get_feedforwards(self, target, error): #maybe add an actual feedforward to handle loads?
        return 0.1

    def raise_lift(self, power):
        self.pid_controller.stop()
        self.state = IntakeLiftState.RAISING
        self.power = abs(power)

    def lower_lift(self, power):
        self.pid_controller.stop()
        self.state = IntakeLiftState.LOWERING
        self.power = abs(power)

    def send_down(self):
        self.position = IntakeLiftPosition.LOWERED
        self.state = IntakeLiftState.PID_CONTROLLED

    def send_up(self):
        self.position = IntakeLiftPosition.RAISED
        self.state = IntakeLiftState.PID_CONTROLLED

    def set_match_start(self):
        self.position = IntakeLiftPosition.MATCH_START
        self.state = IntakeLiftState.PID_CONTROLLED

    def stop(self):
        self.pid_controller.stop()
        self.state = IntakeLiftState.STOPPED
        self.power = 0

    def get_ready(self): #maybe rename to get_on_target?
        return self.pid_controller.get_on_target();
    
    def set_position_pid(self, position):
        if self.state != IntakeLiftState.PID_CONTROLLED:
            self.pid_controller.reset()
        self.pid_controller.run_setpoint(position)
        self.state = IntakeLiftState.PID_CONTROLLED
        # self.state = IntakeLiftState.STOPPED

    def set_power_raw(self, power):
        self.power = power

    def execute(self):
        # if self.state == IntakeLiftState.LOWERING:
        #     self.intake_lift_motor.set(
        #         -min(self.power, RobotMap.IntakeLift.max_power_down)
        #     )
        # elif self.state == IntakeLiftState.RAISING:
        #     self.intake_lift_motor.set(
        #         min(self.power, RobotMap.IntakeLift.max_power_up)
        #     )
        # elif self.state == IntakeLiftState.STOPPED:
        #     self.intake_lift_motor.stopMotor()
        
        self.pid_controller.execute()
        #print(self.get_position())

        if self.position == IntakeLiftPosition.RAISED:
            self.pid_controller.run_setpoint(RobotMap.IntakeLift.up_position)
        elif self.position == IntakeLiftPosition.LOWERED:
            self.pid_controller.run_setpoint(RobotMap.IntakeLift.down_position)
        elif self.position == IntakeLiftPosition.MATCH_START:
            self.pid_controller.run_setpoint(-1) #trying to raise it higher so that it stays up

        if self.state == IntakeLiftState.PID_CONTROLLED:
            self.intake_lift_motor.set(self.power)
        else:
            self.intake_lift_motor.set(0)

class IntakeRollerState:
    STOPPED = 0
    INTAKING = 1
    OUTTAKING = 2

class IntakeRoller:
    intake_roller_motor: WPI_TalonSRX

    def __init__(self):
        self.reset_state()

    def reset_state(self):
        self.state = IntakeRollerState.STOPPED

    def reset(self):
        self.reset_state()

    def intake(self):
        self.state = IntakeRollerState.INTAKING

    def outtake(self):
        self.state = IntakeRollerState.OUTTAKING

    def stop(self):
        self.state = IntakeRollerState.STOPPED

    def execute(self):
        if self.state == IntakeRollerState.INTAKING:
            self.intake_roller_motor.set(-RobotMap.IntakeRoller.roller_power)
        elif self.state == IntakeRollerState.OUTTAKING:
            self.intake_roller_motor.set(RobotMap.IntakeRoller.roller_power)
        elif self.state == IntakeRollerState.STOPPED:
            self.intake_roller_motor.stopMotor()

class Intake:
    # NOTE honestly this is going to be a waste of time coordinating the two subsystems if there's no closed loop control
    # i don't really see a need to do so at this point anyways. i'll leave this for the future then

    intake_lift: IntakeLift
    intake_roller: IntakeRoller

    def __init__(self):
        pass

    def reset_state(self):
        pass

    def reset(self):
        self.intake_lift.reset()
        self.intake_roller.reset()

    def execute(self):
        pass