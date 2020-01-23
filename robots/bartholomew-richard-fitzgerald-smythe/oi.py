from wpilib import XboxController

import math

from robotmap import RobotMap

def deadzone(i, dz):
    return 0 if abs(i) <= dz else i

class Driver:
    def __init__(self, controller: XboxController):
        self.controller = controller

    def get_curvature_output(self):
        x = self.controller.getX()
        y = self.controller.getY(XboxController.Hand.kLeft)

        x = -math.copysign(deadzone(x, RobotMap.OI.driver_deadband) ** 2, x)
        y = math.copysign(deadzone(y, RobotMap.OI.driver_deadband) ** 2, y)

        return x, y

class Sysop:
    def __init__(self, controller: XboxController):
        self.controller = controller