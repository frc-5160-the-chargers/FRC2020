import navx

from robotmap import RobotMap

class NavX:
    navx: navx.AHRS

    def __init__(self):
        self.samples = []

    def reset(self):
        self.navx.reset()
        self.samples = []

    def get_angle(self):
        if len(self.samples) == 0:
            return 0
        return sum(self.samples) / len(self.samples)

    def execute(self):
        self.samples.append(self.navx.getAngle())
        if len(self.samples) > RobotMap.NavX.samples_taking:
            self.samples = self.samples[1:]