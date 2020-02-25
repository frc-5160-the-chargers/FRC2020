import wpilib

from wpilib import SmartDashboard as dash

import oi

from components.sensors import *
from components.intake import *

from dash import Tunable

class PidTuningDrivetrainAuto:
    # TODO ensure that you can use joysticks in auto
    # if you can't, this'll have to be refactored back into `robot.py`

    MODE_NAME = "PID Tuning Lift"
    DEFAULT = False

    driver: oi.Driver
    sysop: oi.Sysop

    intake_lift: IntakeLift

    navx: NavX

    tunables: list
    dash_target_lift_position: Tunable

    def on_enable(self):
        self.intake_lift.reset_encoder()

    def on_disable(self):
        pass

    def on_iteration(self, time_elapsed):
        # push tunables
        if self.driver.get_update_telemetry():
            for t in self.tunables:
                t.push()
            self.intake_lift.pid_manager.push_to_dash()

        dash.putNumber("Lift position", self.intake_lift.get_position())
            
        # handle the current pid controller if starting a pid controller
        if self.driver.get_enable_pid():
            self.intake_lift.state = IntakeLiftState.PID_CONTROLLED

        # update values from dash if requested
        if self.driver.get_update_pid_dash():
            self.intake_lift.pid_manager.update_from_dash()

        if self.intake_lift.state == IntakeLiftState.PID_CONTROLLED:
            self.intake_lift.set_position_pid(self.dash_target_lift_position.get(0))

        # revert to manual control if enabled
        if self.driver.get_manual_control_override():
            pass