from wpilib import XboxController

import math

from robotmap import RobotMap

from utils import map_value

def deadzone(i, dz):
    return 0 if abs(i) <= dz else i

class Driver:
    def __init__(self, controller: XboxController):
        self.controller = controller
    
    def ready_straight_assist(self):
        x, _ = self.get_raw_output()
        return abs(x) <= RobotMap.OI.drivetrain_rotation_assist_deadband

    def get_raw_output(self):
        x = self.controller.getX(XboxController.Hand.kRightHand)
        y = self.controller.getY(XboxController.Hand.kLeftHand)
        return x, y

    def get_curvature_output(self):
        x, y = self.get_raw_output()
        x = -math.copysign(abs(deadzone(x, RobotMap.OI.driver_deadband)) ** .5, x)
        y = math.copysign(abs(deadzone(y, RobotMap.OI.driver_deadband)) ** 2, y)
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
    
    def get_intake_outtake(self):
        return self.controller.getAButton()

    def get_intake_intake(self):
        return self.controller.getBButton()

    def process_lift_axis(self, i):
        i = math.copysign(deadzone(i, RobotMap.OI.lift_deadband) ** 2, i)
        if i > 0:
            # lifting
            mapped = map_value(i, 0, 1, 0, RobotMap.IntakeLift.max_power_up)
        else:
            # lowering
            mapped = map_value(i, -1, 0, -RobotMap.IntakeLift.max_power_down, 0)
        return mapped

    def get_intake_lift_axis(self):
        raw_axis = self.controller.getY(XboxController.Hand.kRightHand)
        return self.process_lift_axis(raw_axis)