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
        self.reset_state()

    def reset_state(self):
        self.state = IntakeLiftState.STOPPED
        self.power = 0
        self.target_position = 0

    def reset_encoder(self):
        self.intake_lift_motor.setQuadraturePosition(0)

    def reset(self):
        self.reset_state()
        self.reset_encoder()
        # self.intake_lift_encoder.reset()

    def get_position(self):
        # return self.intake_lift_encoder.getDistance()
        return self.intake_lift_motor.get() * RobotMap.IntakeLift.encoder_distance_per_pulse

    def get_feedforwards(self, target, error):
        return 0

    def raise_lift(self, power):
        self.pid_controller.stop()
        self.state = IntakeLiftState.RAISING
        self.power = abs(power)

    def lower_lift(self, power):
        self.pid_controller.stop()
        self.state = IntakeLiftState.LOWERING
        self.power = abs(power)

    def stop(self):
        self.pid_controller.stop()
        self.state = IntakeLiftState.STOPPED
        self.power = 0
    
    def set_position_pid(self, position):
        if self.state != IntakeLiftState.PID_CONTROLLED:
            self.pid_controller.reset()
        self.pid_controller.run_setpoint(position)
        self.state = IntakeLiftState.PID_CONTROLLED

    def set_power_raw(self, power):
        self.power = power

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

        # if self.state == IntakeLiftState.PID_CONTROLLED:
        #     print(self.power)
        #     self.intake_lift_motor.set(self.power)
        # else:
        #     self.intake_lift_motor.set(0)

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