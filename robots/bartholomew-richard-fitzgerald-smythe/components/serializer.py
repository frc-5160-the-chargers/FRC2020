from rev import CANSparkMax
from magicbot import tunable
from ctre import WPI_TalonSRX
class Serializer:
    serializer_motor : WPI_TalonSRX

    def __init__(self):
        self.power = 0
        self.state = SerializerState.SERIALIZER_OFF

    def reset(self):
        self.power = 0
    
    def turn_on(self):
        if (self.state == SerializerState.SERIALIZER_OFF):
            self.state = SerializerState.SERIALIZER_ON
            self.power = .1
    
    def turn_off(self):
        if (self.state == SerializerState.SERIALIZER_ON):
            self.state = SerializerState.SERIALIZER_OFF
            self.power = 0

    def execute(self):
        serializer_motor.set(self.power)
        #set moter power
        


class SerializerState:
    # 0-9 == Shoot
    SERIALIZER_ON = 0
    SERIALIZER_OFF = 1