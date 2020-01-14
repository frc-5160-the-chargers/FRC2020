import navx

from config import load_config_file

config_data = load_config_file()

class NavX:
    navx: navx.AHRS

    def __init__(self):
        self.samples = []
        self.samples_holding = config_data['subsystem_config']['navx']['samples_taking']

    def reset(self):
        self.navx.reset()

    def get_angle(self):
        if len(self.samples) == 0:
            return 0
        return sum(self.samples) / len(self.samples)

    def execute(self):
        self.samples.append(self.navx.getAngle())
        if len(self.samples) > self.samples_holding:
            self.samples = self.samples[1:]