from wpilib.controller import PIDController

from utils import PIDValue, clamp
from dash import get_pid, put_pid

import math

def ff_constant(constant, target, error):
    return math.copysign(constant, error)

def ff_flywheel(coefficient, target, error):
    return target * coefficient

class SuperPIDController:
    def __init__(self, pid_values: PIDValue, f_in, f_out, f_feedforwards=None, pid_key=None):
        self.ff_enabled = f_feedforwards != None
        if self.ff_enabled:
            ff = f_feedforwards
        else:
            ff = lambda x: 0

        self.pid_dash_enabled = pid_key != None
        if self.pid_dash_enabled:
            self.pid_key = pid_key
        else:
            self.pid_key = ""

        self.pid_values = pid_values

        self.pid_controller = PIDController(0, 0, 0)

        self.f_in = f_in
        self.ff = ff

        self.output_range = (-1, 1)

        self.pid_values.update_controller(self.pid_controller)

    def get_target(self):
        if self.pid_controller.isEnabled():
            return self.pid_controller.getSetpoint()
        else:
            return 0

    def configure_controller(self, output_range=(-1, 1), percent_tolerance=1):
        self.output_range = output_range
        self.pid_controller.setTolerance(percent_tolerance)

    def update_values(self, pid_values: PIDValue):
        self.pid_values = pid_values
        self.pid_values.update_controller(self.pid_controller)

    def update_from_dash(self):
        if self.pid_dash_enabled:
            self.update_values(get_pid(self.pid_key))
    
    def push_to_dash(self):
        if self.pid_dash_enabled:
            put_pid(self.pid_key, self.pid_values)

    def get_on_target(self):
        return self.pid_controller.atSetpoint()

    def calculate_output(self):
        pid_output = self.pid_controller.calculate(self.f_in())
        output = pid_output + self.ff(self.get_target(), pid_output)
        output = clamp(output, self.output_range[0], self.output_range[1])
        return output

    def stop(self):
        self.reset()

    def start(self):
        self.reset()

    def reset(self):
        self.pid_controller.reset()

    def run_setpoint(self, value):
        self.pid_controller.setSetpoint(value)
        self.start()

class PidManager:
    def __init__(self, controllers):
        self.controllers = controllers
    
    def stop_controllers(self):
        for controller in self.controllers:
            controller.stop()
        
    def reset_controllers(self):
        for controller in self.controllers:
            controller.reset()

    def get_on_target(self):
        for controller in self.controllers:
            if not controller.get_on_target():
                return False
        return True

    def update_from_dash(self):
        for controller in self.controllers:
            controller.update_from_dash()
        
    def push_to_dash(self):
        for controller in self.controllers:
            controller.push_to_dash()