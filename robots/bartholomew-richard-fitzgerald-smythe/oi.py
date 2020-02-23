from wpilib import XboxController

import math

from robotmap import RobotMap

def deadzone(i, dz):
    return 0 if abs(i) <= dz else i

class Driver:
    def __init__(self, controller: XboxController):
        self.controller = controller
    
    def ready_straight_assist(self):
        x, _ = self.get_raw_output()
        return abs(x) <= RobotMap.OI.drivetrain_rotation_assist_deadband

    def get_raw_output(self):
        x = self.controller.getX()
        y = self.controller.getY(XboxController.Hand.kLeft)
        return x, y

    def get_curvature_output(self):
        x, y = self.get_raw_output()
        x = -math.copysign(deadzone(x, RobotMap.OI.driver_deadband) ** 2, x)
        y = math.copysign(deadzone(y, RobotMap.OI.driver_deadband) ** 2, y)
        return x, y

    def get_update_pid_dash(self):
        return self.controller.getXButtonPressed()
    
    def get_toggle_pid_type_pressed(self):
        return self.controller.getBackButtonPressed()

    def get_enable_pid(self):
        return self.controller.getAButtonPressed()

    def get_manual_control_override(self):
        return self.controller.getBButtonPressed()

    def get_update_telemetry(self):
        return self.controller.getYButtonPressed()

class Sysop:
    def __init__(self, controller: XboxController):
        self.controller = controller