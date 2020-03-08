from wpilib import XboxController

def deadzone(i, dz):
    return i if abs(i) > dz else 0

class DriverController:
    def __init__(self, controller):
        self.driver_controller = controller

    def get_navx_reset(self):
        return self.driver_controller.getXButtonPressed()
    
    def get_shooter_full(self):
        return self.driver_controller.getAButton()
    
    def get_shooter_axis(self):
        return deadzone(
            self.driver_controller.getY(hand=XboxController.Hand.kRight),
            0.1
        )
    
    def get_neo_axis(self):
        return deadzone(
            self.driver_controller.getY(hand=XboxController.Hand.kLeft),
            0.1
        )

    def get_update_telemetry(self):
        return self.driver_controller.getAButtonPressed()

class SysopController:
    def __init__(self, controller):
        self.sysop_controller = controller