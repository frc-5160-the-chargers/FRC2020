from rev import CANSparkMax

class SparkMotorConfig:
    def __init__(self, voltage_compensation, stall_current_limit, default_mode, ramp_rate):
        self.voltage_compensation = voltage_compensation
        self.stall_limit = stall_current_limit
        self.default_mode = default_mode
        self.ramp_rate = ramp_rate

def config_spark(spark: CANSparkMax, motor_config: SparkMotorConfig):
    spark.enableVoltageCompensation(motor_config.voltage_compensation)

    spark.setSmartCurrentLimit(motor_config.stall_limit)

    spark.setIdleMode(motor_config.default_mode)

    spark.setOpenLoopRampRate(motor_config.ramp_rate)

def clamp(i, mi, ma):
    return min(ma, max(mi, i))