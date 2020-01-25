from utils import MotorConfig, PIDValue

from ctre import WPI_TalonSRX

class RobotMap:
    class OI:
        driver_deadband = .05
        drivetrain_rotation_deadband = 0.1

    class Encoders:
        left_encoder = 5
        right_encoder = 2

        ticks_per_rotation = 4096
        output_gear_ratio = 12
        wheel_diameter = .1525
        velocity_period = .1 # quadvelocity returns units per x seconds

    class Drivetrain:
        motors_left = [5, 6, 7]
        motors_right = [2, 3, 4]

        motor_config = MotorConfig(
            voltage_saturation=11,
            deadband=0,
            peak_current=100,
            continuous_current=40,
            default_mode=WPI_TalonSRX.NeutralMode.Brake,
            ramp_rate=.5,
        )

        kF_turn = .3
        kF_straight = .15

        max_motor_power = .3

        turn_pid = PIDValue(-0.035, 0, -0.15)
        turn_pid_key = "Drivetrain Turn PID"

        position_pid = PIDValue(-0.035, 0, -0.15)
        position_pid_key = "Drivetrian Position PID"

        velocity_left = PIDValue(0, 0, 0)
        velocity_left_key = "Left Velocity PID"

        velocity_right = PIDValue(0, 0, 0)
        velocity_right_key = "Right Velocity PID"

        wheelbase = .55

    class NavX:
        samples_taking = 5