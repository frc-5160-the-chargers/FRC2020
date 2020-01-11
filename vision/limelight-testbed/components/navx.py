import navx

class NavX:
    navx: navx.AHRS

    def __init__(self):
        self.displacement_x = 0
        self.displacement_y = 0
        self.displacement_z = 0

    def reset(self):
        self.reset_displacement()

    def reset_displacement(self):
        self.displacement_x = 0
        self.displacement_y = 0
        self.displacement_z = 0

        self.navx.resetDisplacement()

    def get_displacement(self):
        return (self.displacement_x, self.displacement_y, self.displacement_z)

    def execute(self):
        self.displacement_x = self.navx.getDisplacementX()
        self.displacement_y = self.navx.getDisplacementY()
        self.displacement_z = self.navx.getDisplacementZ()