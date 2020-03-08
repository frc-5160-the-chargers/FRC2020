from wpilib import SmartDashboard as dash

from utils import PIDValue

def put_tuple(key, t, op=lambda x: x):
    dash.putString(key, ", ".join([str(op(i)) for i in t]))

def put_pid(key, pid: PIDValue):
    dash.putNumber(f"{key}_kP", pid.p)
    dash.putNumber(f"{key}_kI", pid.i)
    dash.putNumber(f"{key}_kD", pid.d)
    
def get_pid(key):
    p = dash.getNumber(f"{key}_kP", 0)
    i = dash.getNumber(f"{key}_kI", 0)
    d = dash.getNumber(f"{key}_kD", 0)
    return PIDValue(p, i, d)

class Tunable:
    def __init__(self, key, default_value=0):
        self.value = default_value
        self.key = key

    def set(self, value):
        self.value = value
        dash.putNumber(self.key, value)

    def get(self, default_value=None):
        if default_value == None:
            return dash.getNumber(self.key, self.value)
        else:
            return dash.getNumber(self.key, default_value)

    def push(self):
        dash.putNumber(self.key, self.value)