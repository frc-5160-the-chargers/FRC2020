import math

class ArcDrive:
    def __init__(self, radius, theta, v_max, wheelbase):
        if radius <= 0 or wheelbase/2 >= radius:
            self.valid = False
            return

        self.valid = True

        self.radius = radius
        self.theta = theta
        self.v_max = v_max

        w = wheelbase
        r = radius
        theta = math.radians(theta)
        
        if theta < 0:
            # turn left
            l_l = (r + w / 2) * theta
            l_r = (r - w / 2) * theta

            v_l = v_max
            v_r = ((r - w / 2) / (r + w / 2)) * v_max
        else:
            # turn rigt
            l_l = (r - w / 2) * theta
            l_r = (r + w / 2) * theta

            v_l = ((r - w / 2) / (r + w / 2)) * v_max
            v_r = v_max

        t = l_l / v_l

        d_theta_d_t = theta / t

        self.v_l = v_l
        self.v_r = v_r

        self.l_l = l_l
        self.l_r = l_r

        self.d_theta_d_t = math.degrees(d_theta_d_t)
        self.t = t
    
    def __str__(self):
        return f"Velocities: ({self.v_l}m/s, {self.v_r}m/s)\nDistances: ({self.l_l}m, {self.l_r}m)\nTime: {self.t}s\nGyro Rate: {self.d_theta_d_t} deg/s"