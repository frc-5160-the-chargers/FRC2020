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

        #Tuples: angle to turn to and distance to drive (angle, dist)
        self.nodes = []
        prev_pt = traj_pts[0]
        for pt in traj_pts[1:]:
            self.nodes.append([self.calc_angle(prev_pt, pt), math.dist(prev_pt, pt)])
            prev_pt = pt

        self.next_state('track')
    
    @state()
    def track(self):
        index = 0
        self.drivetrain.powertrain.set_arcade_powers(power=0.1)
        while index < len(self.nodes):
            self.drivetrain.turn_to_angle(self.nodes[index][0])
            if self.drivetrain.get_position(EncoderSide.BOTH) > self.nodes[index][1]
                index+=1
                self.drivetrain.encoders.reset()
        while not self.drivetrain.turn_pid.get_on_target():
            continue
        self.drivetrain.stop()
        self.done()