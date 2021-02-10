from ctre import WPI_TalonSRX
from components.limelight import Limelight
class Shooter:
    shooter_srx: WPI_TalonSRX
    limelight : Limelight

    def __init__(self):
        self.power = 0
        self.target_rpm = 0
        self.shooter_kP = 0
        self.shooter_kF = 0

    def get_raw_velocity(self):
        return self.shooter_srx.get_velocity()

    def update_pid(self, kP, kF):
        self.shooter_kP = kP
        self.shooter_kF = kF

    def set_rpm(self, rpm):
        self.target_rpm = rpm

    def update_motor_velocity(self):
        rpm_error = self.get_raw_velocity() - self.target_rpm
        kP = self.shooter_kP * rpm_error
        kF = self.target_rpm * self.shooter_kF
        self.power = kP + kF

    def reset(self):
        self.target_rpm = 0
        self.power = 0


    def fire(self):
        self.set_rpm(600)#formula
        #stop if fired all

    def execute(self):
        self.update_motor_velocity()

        self.shooter_srx.set_power(self.power)