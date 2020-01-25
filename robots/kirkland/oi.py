from wpilib import XboxController

from robotmap import RobotMap

import math

def deadzone(i, dz):
    return i if abs(i) > dz else 0

class DriverController:
    def __init__(self, controller):
        self.driver_controller = controller
    
    def get_aided_rotation(self):
        return abs(self.driver_controller.getX()) <= RobotMap.OI.drivetrain_rotation_deadband

    def get_driver_input_curvature(self):
        x = self.driver_controller.getX()
        y = self.driver_controller.getY(XboxController.Hand.kLeft)

        x = -math.copysign(deadzone(x, RobotMap.OI.driver_deadband) ** 2, x)
        y = math.copysign(deadzone(y, RobotMap.OI.driver_deadband) ** 2, y)

        return x, y

    def get_update_pid_dash(self):
        return self.driver_controller.getXButtonPressed()
    
    def get_toggle_pid_type_pressed(self):
        return self.driver_controller.getBackButtonPressed()

    def get_enable_pid(self):
        return self.driver_controller.getAButtonPressed()

    def get_manual_control_override(self):
        return self.driver_controller.getBButtonPressed()

    def get_update_telemetry(self):
        return self.driver_controller.getYButtonPressed()

class SysopController:
    def __init__(self, controller):
        self.sysop_controller = controller