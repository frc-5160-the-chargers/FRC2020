from utils import MotorConfig, PIDValue

from ctre import WPI_TalonSRX

class RobotMap:
    class OI:
        driver_deadband = .05
        drivetrain_rotation_deadband = 0.1

    class Encoders:
        enabled = True

        left_encoder = 5
        right_encoder = 2

        ticks_per_rotation = 4096
        wheel_diameter = 6

    class Drivetrain:
        enabled = True

        motors_left = [5, 6, 7]
        motors_right = [2, 3, 4]

        motor_config = MotorConfig(
            voltage_saturation=11,
            deadband=.025,
            peak_current=100,
            continuous_current=40,
            default_mode=WPI_TalonSRX.NeutralMode.Brake
        )

        turn_pid = PIDValue(0, 0, 0)
        turn_pid_key = "Drivetrain Turn PID"

        position_pid = PIDValue(0, 0, 0)
        position_pid_key = "Drivetrian Position PID"

    class NavX:
        enabled = True

        samples_taking = 5