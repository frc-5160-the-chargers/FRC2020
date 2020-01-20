from wpilib import SmartDashboard as dash

from utils import PIDValue

def put_tuple(key, t, op=lambda x: x):
    dash.putString(key, ", ".join([str(op(i)) for i in t]))

def put_pid(key, pid: PIDValue):
    dash.putString(key, f"P [{pid.p}], I [{pid.i}], D [{pid.d}]")

def get_pid(key):
    p = dash.getNumber(f"{key}_kP")
    i = dash.getNumber(f"{key}_kI")
    d = dash.getNumber(f"{key}_kD")
    return PIDValue(p, i, d)