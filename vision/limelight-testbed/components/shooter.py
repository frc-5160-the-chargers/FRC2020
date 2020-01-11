from ctre import WPI_TalonSRX

class Shooter:
    shooter_srx_a: WPI_TalonSRX
    shooter_srx_b: WPI_TalonSRX

    def __init__(self):
        self.power = 0

    def reset(self):
        self.power = 0

    def execute(self):
        self.shooter_srx_a.set(self.power)
        self.shooter_srx_b.set(self.power)