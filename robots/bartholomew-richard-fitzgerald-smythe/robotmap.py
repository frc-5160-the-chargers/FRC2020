from utils import SparkMotorConfig, PIDValue, TalonMotorConfig

from ctre import NeutralMode
from rev import IdleMode

import math

class RobotMap:
    class OI:
        driver_deadband = .05
        drivetrain_rotation_assist_deadband = .05

        lift_deadband = .05
        climb_deadband = .1

    class Drivetrain:
        motors_left = [1, 2]
        motors_right = [3, 4]

        # TODO tune this
        # https://www.chiefdelphi.com/t/any-tips-on-neo-motors/355988/8
        # this looks like a good place to start for that
        # @40A the motor will be able to stall for an entire match and not burn out
        # https://www.revrobotics.com/neo-brushless-motor-locked-rotor-testing/
        motor_config = SparkMotorConfig(
            voltage_compensation=11,
            stall_current_limit=39,
            default_mode=IdleMode.kBrake,
            ramp_rate=1,
            reverse_motor=True
        )

        max_motor_power = .35

        # TODO all of these PID values are taken from kirkland
        kF_turn = .3
        kF_straight = .15
        kF_velocity = .2

        turn_pid = PIDValue(-0.03, 0, 0)
        turn_pid_key = "Drivetrain Turn PID"

        position_pid = PIDValue(-0.035, 0, -0.001)
        position_pid_key = "Drivetrian Position PID"

        velocity_left = PIDValue(0, 0, 0)
        velocity_left_key = "Left Velocity PID"

        velocity_right = PIDValue(0, 0, 0)
        velocity_right_key = "Right Velocity PID"
    
    class IntakeLift:
        motor_port = 5
        max_power = .4

        max_power_up = .6
        max_power_down = .6

        motor_config = TalonMotorConfig(
            voltage_saturation=11,
            deadband=0.05,
            peak_current=60,
            continuous_current=39,
            default_mode=NeutralMode.Brake,
            ramp_rate=.2,
            reverse_motor=True
        )

        encoder_distance_per_pulse = 360/4096 * (24/60)
        encoder_averaging = 5

        pid_values = PIDValue(0.04, 0, 0.005)
        pid_key = "Intake Lift PID"

        encoder_port_a = 4
        encoder_port_b = 5

        up_position = 10
        down_position = 76

    class IntakeRoller:
        motor_port = 6
        max_power = .4

        roller_power = .6

        motor_config = TalonMotorConfig(
            voltage_saturation=11,
            deadband=0.05,
            peak_current=60,
            continuous_current=39,
            default_mode=NeutralMode.Brake,
            ramp_rate=.2
        )

    class Climber:
        motor_port = 8

        max_power = .8
        
        motor_config = TalonMotorConfig(
            voltage_saturation=11,
            deadband=.05,
            peak_current=60,
            continuous_current=39,
            default_mode=NeutralMode.Brake,
            ramp_rate=0
        )
    
    class ColorWheel:
        motor_port = 7

    class Encoders:
        wheel_diameter = 7.4
        distance_per_pulse = (wheel_diameter*math.pi)/360
        
        left_encoder_a = 0
        left_encoder_b = 1

        right_encoder_a = 2
        right_encoder_b = 3