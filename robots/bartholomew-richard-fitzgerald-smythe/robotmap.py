from utils import SparkMotorConfig, PIDValue

from rev import IdleMode

class RobotMap:
    class OI:
        driver_deadband = .05
        drivetrain_rotation_deadband = .1

    class Drivetrain:
        motors_left = [1, 2]
        motors_right = [3, 4]

        # TODO tune this
        motor_config = SparkMotorConfig(
            voltage_compensation=11,
            stall_current_limit=15,
            default_mode=IdleMode.kBrake,
            ramp_rate=.5
        )

        max_motor_power = .3

        # TODO all of these PID values are taken from kirkland
        kF_turn = .3
        kF_straight = .15
        kF_velocity = .2

        turn_pid = PIDValue(-0.035, 0, -0.15)
        turn_pid_key = "Drivetrain Turn PID"

        position_pid = PIDValue(-0.035, 0, -0.15)
        position_pid_key = "Drivetrian Position PID"

        velocity_left = PIDValue(0, 0, 0)
        velocity_left_key = "Left Velocity PID"

        velocity_right = PIDValue(0, 0, 0)
        velocity_right_key = "Right Velocity PID"

    class Encoders:
        distance_per_pulse = 1 # TODO get correct value
        
        left_encoder_a = 0
        left_encoder_b = 1

        right_encoder_a = 2
        right_encoder_b = 3