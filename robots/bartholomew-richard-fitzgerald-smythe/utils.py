from rev import CANSparkMax
from ctre import WPI_TalonSRX

from wpilib import PIDController

class SparkMotorConfig:
    def __init__(self, voltage_compensation, stall_current_limit, default_mode, ramp_rate, reverse_motor=False):
        self.voltage_compensation = voltage_compensation
        self.stall_limit = stall_current_limit
        self.default_mode = default_mode
        self.ramp_rate = ramp_rate
        self.reversed = reverse_motor

def config_spark(spark: CANSparkMax, motor_config: SparkMotorConfig):
    spark.restoreFactoryDefaults()
    spark.enableVoltageCompensation(motor_config.voltage_compensation)
    spark.setSmartCurrentLimit(motor_config.stall_limit)
    spark.setIdleMode(motor_config.default_mode)
    spark.setOpenLoopRampRate(motor_config.ramp_rate)
    spark.setClosedLoopRampRate(motor_config.ramp_rate)
    spark.setInverted(motor_config.reversed)

class TalonMotorConfig:
    def __init__(self, voltage_saturation, deadband, peak_current, continuous_current, default_mode, ramp_rate, reverse_motor=False):
        self.voltage_saturation = voltage_saturation
        self.deadband = deadband
        self.peak_current = peak_current
        self.continuous_current = continuous_current
        self.default_mode = default_mode
        self.ramp_rate = ramp_rate
        self.reversed = reverse_motor

def config_talon(talon: WPI_TalonSRX, motor_config: TalonMotorConfig):
    # when deploying it might be a somewhat good idea to actually uncomment this next line
    # talon.configFactoryDefault()

    talon.enableVoltageCompensation(True)
    talon.configVoltageCompSaturation(motor_config.voltage_saturation)

    talon.enableCurrentLimit(True)
    talon.configPeakCurrentLimit(motor_config.peak_current)
    talon.configContinuousCurrentLimit(motor_config.continuous_current)

    talon.setNeutralMode(motor_config.default_mode)

    talon.configNeutralDeadband(motor_config.deadband)

    talon.configOpenLoopRamp(motor_config.ramp_rate)

    talon.setInverted(motor_config.reversed)

def clamp(i, mi, ma):
    return min(ma, max(mi, i))

def average(l):
    return sum(l)/len(l)

def map_value(i, initial_min, initial_max, new_min, new_max):
    # perform a linear mapping of a value across spaces
    i = clamp(i, initial_min, initial_max)
    i += initial_min
    distance = initial_max - initial_min
    percent = i/distance
    new_distance = new_max - new_min
    out = percent * new_distance
    out -= new_min
    return out

class PIDValue:
    def __init__(self, p, i, d):
        self.p = p
        self.i = i
        self.d = d

    def update_controller(self, controller: PIDController) -> None:
        controller.P = self.p
        controller.I = self.i
        controller.D = self.d
