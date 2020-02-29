from ctre import WPI_TalonSRX
from components.sensors import WheelOfFortuneColor, WheelOfFortuneSensor
from robotmap import RobotMap

class ColorWheelState:
    MANUAL = 0
    POSITION = 1
    ROTATION = 2
    ON_TARGET = 3

class ColorWheelController:
    i2c_color_sensor: WheelOfFortuneSensor

    color_wheel_motor: WPI_TalonSRX

    def __init__(self):
        self.reset_state()

    def reset_state(self):
        self.state = ColorWheelState.MANUAL
        self.power = 0

    def stop(self):
        self.state = ColorWheelState.MANUAL
        self.power = 0

    def manual_power(self, power):
        self.state = ColorWheelState.MANUAL
        self.power = power
        
    def position_control(self, target_color):
        # check and see if on target color
        current_color = self.color_sensor.get_color()
        if current_color == target_color:
            self.state = ColorWheelState.ON_TARGET
            self.power = 0
            return

        # s p i n
        self.state = ColorWheelState.POSITION
        self.power = RobotMap.ColorWheel.spinning_power_position

    def rotation_control(self):
        # TODO make this more automated and not in a sus way like before
        # i'd suggest running it for a time maybe? otherwise we'd need a sensor filter
        self.state = ColorWheelState.ROTATION
        self.power = RobotMap.ColorWheel.spinning_power_rotation    

    def execute(self):
        self.color_wheel_motor.set(self.power)