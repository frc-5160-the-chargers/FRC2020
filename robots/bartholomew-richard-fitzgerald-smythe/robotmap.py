from utils import SparkMotorConfig

from rev import IdleMode

class RobotMap:
    class OI:
        driver_deadband = .05

    class Drivetrain:
        motors_left = [1, 2]
        motors_right = [3, 4]

        motor_config = SparkMotorConfig(
            voltage_compensation=11,
            stall_current_limit=15,
            default_mode=IdleMode.kBrake,
            ramp_rate=.5
        )

        max_motor_power = .3