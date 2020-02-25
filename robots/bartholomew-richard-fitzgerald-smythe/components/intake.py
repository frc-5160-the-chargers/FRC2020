from wpilib import Encoder
from ctre import WPI_TalonSRX

from robotmap import RobotMap

from utils import clamp
from pid import SuperPIDController

class IntakeLiftState:
    STOPPED = 0
    RAISING = 1
    LOWERING = 2

class IntakeLift:
    intake_lift_motor: WPI_TalonSRX
    intake_lift_encoder: Encoder

    def __init__(self):
        self.reset_state()
        self.pid_controller = SuperPIDController(
            pid_values=RobotMap.IntakeLift.pid_values,
            f_in=self.get_position,
            f_out=lambda x: self.set_power_raw(x),
            f_feedforwards=lambda a, b: 0,
            pid_key=RobotMap.IntakeLift.pid_key
        )

    def reset_state(self):
        self.state = IntakeLiftState.STOPPED
        self.power = 0

    def reset(self):
        self.reset_state()
        self.intake_lift_encoder.setDistancePerPulse(RobotMap.IntakeLift.encoder_distance_per_pulse)

    def get_position(self):
        return self.intake_lift_encoder.getDistance()

    def get_feedforwards(self):
        pass

    def raise_lift(self, power):
        self.state = IntakeLiftState.RAISING
        self.power = abs(power)

    def lower_lift(self, power):
        self.state = IntakeLiftState.LOWERING
        self.power = abs(power)

    def stop(self):
        self.state = IntakeLiftState.STOPPED
        self.power = 0

    def set_power_raw(self, power):
        if self.power > 0:
            self.raise_lift(power)
        elif self.power < 0:
            self.lower_lift(power)
        else:
            self.stop()

    def execute(self):
        if self.state == IntakeLiftState.LOWERING:
            self.intake_lift_motor.set(
                -min(self.power, RobotMap.IntakeLift.max_power_down)
            )
        elif self.state == IntakeLiftState.RAISING:
            self.intake_lift_motor.set(
                min(self.power, RobotMap.IntakeLift.max_power_up)
            )
        elif self.state == IntakeLiftState.STOPPED:
            self.intake_lift_motor.stopMotor()

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
            self.intake_roller_motor.set(RobotMap.IntakeRoller.roller_power)
        elif self.state == IntakeRollerState.OUTTAKING:
            self.intake_roller_motor.set(-RobotMap.IntakeRoller.roller_power)
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