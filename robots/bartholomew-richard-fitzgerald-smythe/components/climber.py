from ctre import WPI_TalonSRX

class Climber:
    climber_motor: WPI_TalonSRX

    def __init__(self):
        pass

    def reset_state(self):
        self.power = 0

    def reset(self):
        self.reset_state()

    def set_power(self, power):
        self.power = power

    def execute(self):
        self.climber_motor.set(self.power)