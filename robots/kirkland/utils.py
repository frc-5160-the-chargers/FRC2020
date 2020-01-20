from ctre import WPI_TalonSRX
from wpilib import PIDController

class PIDValue:
    def __init__(self, p, i, d):
        self.p = p
        self.i = i
        self.d = d

    def update_controller(self, controller: PIDController) -> None:
        controller.P = self.p
        controller.I = self.i
        controller.D = self.d

class MotorConfig:
    def __init__(self, voltage_saturation, deadband, peak_current, continuous_current, default_mode):
        self.voltage_saturation = voltage_saturation
        self.deadband = deadband
        self.peak_current = peak_current
        self.continuous_current = continuous_current
        self.default_mode = default_mode

def config_talon(talon: WPI_TalonSRX, motor_config: MotorConfig) -> None:
    talon.enableVoltageCompensation(True)
    talon.configVoltageCompSaturation(motor_config.voltage_saturation)

    talon.enableCurrentLimit(True)
    talon.configPeakCurrentLimit(motor_config.peak_current)
    talon.configContinuousCurrentLimit(motor_config.continuous_current)

    talon.setNeutralMode(motor_config.default_mode)

    talon.configNeutralDeadband(motor_config.deadband)