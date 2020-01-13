from wpilib import SmartDashboard as dash

def put_tuple(key, t, op=lambda x: x):
    dash.putString(key, ", ".join([str(op(i)) for i in t]))