import wpilib

import oi

from components.drivetrain import *
from components.sensors import *

from dash import Tunable

class PidTuningAuto:
    # TODO ensure that you can use joysticks in auto
    # if you can't, this'll have to be refactored back into `robot.py`

    MODE_NAME = "PID Tuning"
    DEFAULT = True

    driver: oi.Driver
    sysop: oi.Sysop

    drivetrain: Drivetrain

    pid_mode: int
    dash_target_angle: Tunable
    dash_target_position: Tunable
    dash_target_vel_left: Tunable
    dash_target_vel_right: Tunable

    def on_enable(self):
        self.drivetrain.reset()

    def on_disable(self):
        self.drivetrain.reset()

    def on_iteration(self, time_elapsed):
        # handle the current pid controller if starting a pid controller
        if self.driver.get_enable_pid():
            if self.pid_mode == DrivetrainState.PID_TURNING:
                self.drivetrain.turn_to_angle(self.dash_target_angle.get())
            if self.pid_mode == DrivetrainState.PID_STRAIGHT:
                self.drivetrain.drive_to_position(self.dash_target_position.get())
            if self.pid_mode == DrivetrainState.PID_VELOCITY:
                self.drivetrain.velocity_control(self.dash_target_vel_left.get(), self.dash_target_vel_right.get())

        # rotate through PID control types
        if self.driver.get_toggle_pid_type_pressed():
            self.pid_mode = {
                DrivetrainState.PID_STRAIGHT: DrivetrainState.PID_TURNING,
                DrivetrainState.PID_TURNING: DrivetrainState.PID_VELOCITY,
                DrivetrainState.PID_VELOCITY: DrivetrainState.PID_STRAIGHT
            }[self.pid_mode]

        # update values from dash if requested
        if self.driver.get_update_pid_dash():
            self.drivetrain.pid_manager.update_from_dash()

        # NOTE this is simply copied and pasted from robot.py as of (2/22/2020)
        if self.drivetrain.state == DrivetrainState.MANUAL_DRIVE:
            driver_x, driver_y = self.driver.get_curvature_output()
            self.drivetrain.curvature_drive(driver_y, driver_x)
        elif self.pid_mode == DrivetrainState.PID_TURNING:
            self.drivetrain.turn_to_angle(self.dash_target_angle.get())
        elif self.pid_mode == DrivetrainState.PID_STRAIGHT:
            self.drivetrain.drive_to_position(self.dash_target_position.get())
        elif self.pid_mode == DrivetrainState.PID_VELOCITY:
            self.drivetrain.velocity_control(self.dash_target_vel_left.get(), self.dash_target_vel_right.get())

        # revert to manual control if enabled
        if self.driver.get_manual_control_override():
            self.drivetrain.state = DrivetrainState.MANUAL_DRIVE