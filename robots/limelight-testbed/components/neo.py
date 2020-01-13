from rev import CANSparkMax

class Neo:
    neo_motor: CANSparkMax

    def __init__(self):
        self.power = 0

    def reset(self):
        self.power = 0

    def execute(self):
        self.neo_motor.set(self.power)