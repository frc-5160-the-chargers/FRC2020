from wpilib import Encoder

from navx import AHRS

from utils import average
from robotmap import RobotMap

class EncoderSide:
    BOTH = 0
    LEFT = 1
    RIGHT = 2

class Encoders:
    left_encoder: Encoder
    right_encoder: Encoder

    def __init__(self):
        pass

    def reset_state(self):
        pass

    def reset(self):
        self.reset_state()
        self.left_encoder.reset()
        self.right_encoder.reset()

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

    def __init__(self):
        pass

    def reset_state(self):
        pass

    def reset(self):
        self.reset_state()

        self.navx_ahrs.reset()

    def get_heading(self):
        return self.navx_ahrs.getAngle()

    def execute(self):
        pass