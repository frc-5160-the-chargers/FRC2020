import navx
import time

class NavX:
    navx: navx.AHRS

    def __init__(self):
        self.reset_counters()

    def reset(self):
        self.reset_counters()

    def reset_counters(self):
        self.displacement = (0, 0, 0)
        self.velocity = (0, 0, 0)
        self.acceleration = (0, 0, 0)
        self.last_time = -1

    def get_displacement(self):
        return self.displacement

    def execute(self):
        ax = self.navx.getRawAccelX()
        ay = self.navx.getRawAccelY()
        az = self.navx.getRawAccelZ()

        if self.last_time == -1:
            self.last_time = time.time()
        
        dt = time.time()-self.last_time
        
        self.acceleration = (
            ax,
            ay,
            az
        )

        self.velocity = (
            self.velocity[0] + self.acceleration[0]*dt,
            self.velocity[1] + self.acceleration[1]*dt,
            self.velocity[2] + self.acceleration[2]*dt
        )

        self.displacement = (
            self.displacement[0] + self.velocity[0]*dt,
            self.displacement[1] + self.velocity[1]*dt,
            self.displacement[2] + self.velocity[2]*dt
        )

        self.last_time += dt