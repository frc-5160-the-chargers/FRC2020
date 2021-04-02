from wpilib import Encoder

from navx import AHRS



from rev.color import ColorSensorV3

from utils import average
from robotmap import RobotMap

class EncoderSide:
    BOTH = 0
    LEFT = 1
    RIGHT = 2

class Encoders:
    left_encoder: Encoder
    right_encoder: Encoder

    #location: PosApprox

    def __init__(self):
        pass

    def reset_state(self):
        pass

    def reset(self):
        self.reset_state()
        self.left_encoder.reset()
        self.right_encoder.reset()

        self.location.encoder_reset();

        self.set_distance_per_pulse(RobotMap.Encoders.distance_per_pulse)

    def set_distance_per_pulse(self, distance_per_pulse):
        for encoder in [self.left_encoder, self.right_encoder]:
            encoder.setDistancePerPulse(distance_per_pulse)

    def get_encoders_side(self, encoder_side: EncoderSide):
        return {
            EncoderSide.LEFT: [self.left_encoder],
            EncoderSide.RIGHT: [self.right_encoder],
            EncoderSide.BOTH: [self.left_encoder, self.right_encoder]
        }[encoder_side]

    def get_position(self, encoder_side: EncoderSide):
        encoders_querying = self.get_encoders_side(encoder_side)
        return average([encoder.getDistance() for encoder in encoders_querying])
    
    def get_velocity(self, encoder_side: EncoderSide):
        encoders_querying = self.get_encoders_side(encoder_side)
        return average([encoder.getRate() for encoder in encoders_querying])

    def execute(self):
        pass

class NavX:
    navx_ahrs: AHRS
    #location: PosApprox

    def __init__(self):
        self.reset_state()

    def reset_state(self):
        pass

    def reset(self):
        self.reset_state()

#        self.location.navx_reset();

        self.navx_ahrs.reset()

    def get_heading(self): #in degrees
        return self.navx_ahrs.getAngle()

    def execute(self):
        pass

class WheelOfFortuneColor:
    NONE = "none"
    BLUE = "blue"
    GREEN = "green"
    RED = "red"
    YELLOW = "yellow"

class WheelOfFortuneSensor:
    # i feel like this is a logic bomb lul
    i2c_color_sensor: ColorSensorV3
    
    COLOR_VALUES = [(0,255,255),(0,255,0),(255,0,0),(0,0,255)]
    COLOR_NAMES = [WheelOfFortuneColor.BLUE, WheelOfFortuneColor.GREEN, WheelOfFortuneColor.RED, WheelOfFortuneColor.YELLOW] # counterclockwise order

    def __init__(self):
        pass

    def execute(self):
        pass

    def nearest_color(self, rgb):
        differences = [
            sum([
                abs(rgb[i] - color[i]) for i in range(3)
            ]) for color in self.COLOR_VALUES
        ]
        return self.COLOR_NAMES[differences.index(min(differences))]
        
    def get_rgb(self):
        color_raw = self.i2c_color_sensor.getColor()
        return (color_raw.red, color_raw.blue, color_raw.green)

    def get_ir(self):
        return self.i2c_color_sensor.getIR()
    
    def get_color(self):
        return self.nearest_color(self.get_rgb())