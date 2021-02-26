from magicbot import AutonomousStateMachine, state

from components.drivetrain import Drivetrain, DrivetrainState
from components.sensors import EncoderSide

import numpy as np
import math

from scipy import interpolate

class Autonav(AutonomousStateMachine):
    MODE_NAME = "Autonav"

    drivetrain: Drivetrain

    #Angles in degrees
    def calc_angle(self, pt1, pt2):
        dx = pt2[0] - pt1[0]
        dy = pt2[1] - pt1[1]

        rad_ang = math.atan2(dy,dx)

        return math.degrees(rad_ang)

    @state(first=True)
    def get_angles(self):
        #Distances in inches
        x = (0,36,72,108,144,180)
        y = (0,36,0,0,0,0)
        
        data = np.array((x,y))

        resolution = 50

        tck,_ = interpolate.splprep(data, s=0)
        plot_pts = np.arange(0, 1.01, 1/resolution)
        out = interpolate.splev(plot_pts, tck)

        traj_pts = []
        for i in range(len(out[0])):
            traj_pts.append([out[0][i], out[1][i]])

        self.angles = []
        self.distance = 0
        prev_pt = traj_pts[0]
        for pt in traj_pts[1:]:
            self.angles.append(self.calc_angle(prev_pt, pt))
            self.distance += math.sqrt((pt[0]-prev_pt[0])+(pt[1]-prev_pt[1]))
            prev_pt = pt

        self.distance /= resolution
        self.spacing = abs(self.distance) #This is just to copy by value rather than reference, abs isn't mathematically important

        self.next_state('track')
    
    @state()
    def track(self):
        index = 0
        self.drivetrain.drive_straight(0.1)
        while index < len(self.angles):
            self.drivetrain.turn_to_angle(self.angles[index])
            if self.drivetrain.get_position() > self.distance
                index+=1
                self.distance+=self.spacing
        while not self.drivetrain.turn_pid.get_on_target():
            continue
        self.drivetrain.stop()
        self.done()