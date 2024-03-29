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

        color_wheel_deadband = .1

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

        distance_between_wheels = 22.5

        max_motor_power = .4
        max_auto_power = .7
        turbo_mode_power = .57

        #kF = feedforward constants - base loads that need to be provided when on target (estimation). kF_straightz
        kF_straight = .15
        kF_turn = 0

        # TODO retune
        turn_pid = PIDValue(-0.1, 0, -0.01)
        turn_pid_key = "Drivetrain Turn PID"

        position_pid = PIDValue(-0.035, 0, -0.001)
        position_pid_key = "Drivetrian Position PID"

        #TODO: DEFINETELY retune IMMEDIATELY
        limelight_turn_pid = PIDValue(0.2, 0.001, 0.1)
        limelight_turn_pid_key = "Drivetrain Limelight Turning PID"

        limelight_distance_pid = PIDValue(0.035, 0, 0.001)
        limelight_distance_pid_key = "Drivetrain Limelight Distance Driving PID"
    
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

        encoder_distance_per_pulse = 360/4096 * (24/60) #in inches
        encoder_averaging = 5

        pid_values = PIDValue(0.04, 0, 0.005)
        pid_key = "Intake Lift PID"

        encoder_port_a = 4
        encoder_port_b = 5

        up_position = 2
        down_position = 77

    class IntakeRoller:
        motor_port = 6
        max_power = .4

        roller_power = .3

        motor_config = TalonMotorConfig(
            voltage_saturation=11,
            deadband=0.05,
            peak_current=60,
            continuous_current=39,
            default_mode=NeutralMode.Brake,
            ramp_rate=.3
        )

    class ColorWheel:
        motor_port = 7

        motor_config = TalonMotorConfig(
            voltage_saturation=11,
            deadband=.05,
            peak_current=60,
            continuous_current=39,
            default_mode=NeutralMode.Brake,
            ramp_rate=0.2
        )

        spinning_power_position = 0.5
        spinning_power_rotation = .7

        max_power = .9

    class Climber:
        motor_port = 8

        max_power = .9
        
        motor_config = TalonMotorConfig(
            voltage_saturation=11,
            deadband=.05,
            peak_current=60,
            continuous_current=39,
            default_mode=NeutralMode.Brake,
            ramp_rate=0.25
        )
    
    class Shooter:
        motor_port = 8
        max_power = .6

        motor_config = TalonMotorConfig(
            voltage_saturation=11,
            deadband=.05,
            peak_current=60,
            continuous_current=39,
            default_mode=NeutralMode.Brake,
            ramp_rate=0.25
        )

        target_height = 8*12; #modify 

    class Serializer:
        motor_port = -1
        max_power = .1

        motor_config = TalonMotorConfig(
            voltage_saturation=11,
            deadband=0.05,
            peak_current=60,
            continuous_current=39,
            default_mode=NeutralMode.Brake,
            ramp_rate=.2,
            reverse_motor=True
        )


    class Encoders:
        wheel_diameter = 7.4
        distance_per_pulse = (wheel_diameter*math.pi)/360
        
        left_encoder_a = 0
        left_encoder_b = 1

        right_encoder_a = 2
        right_encoder_b = 3

    class Limelight:
        mount_height = 0.45  # meters - TEMPORARY
        mount_angle = 0  # degrees

        target_height = 2.3  # meters

        enabled_devices = [
            "shooter",
            "camera"
        ]
