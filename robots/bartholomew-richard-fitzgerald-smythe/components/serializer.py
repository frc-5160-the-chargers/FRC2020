from rev import CANSparkMax
from magicbot import tunable
class Serializer:
    
    def __init__(self):
        self.power = 0
        self.state = SerializerState.SERIALIZER_OFF

    def reset(self):
        self.power = 0
    
    def turn_on(self):
        if (self.state == SerializerState.SERIALIZER_OFF):
            self.state = SerializerState.SERIALIZER_ON
    
    def turn_off(self):
        if (self.state == SerializerState.SERIALIZER_OFF):
            self.state = SerializerState.SERIALIZER_ON

    def execute(self):
        pass
        #set moter power
        


class SerializerState:
    # 0-9 == Shoot
    SERIALIZER_ON = 0
    SERIALIZER_OFF = 1