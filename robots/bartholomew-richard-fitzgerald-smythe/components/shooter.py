from rev import CANSparkMax
from ctre import WPI_TalonSRX
from magicbot import tunable
import math
from robotmap import RobotMap
from components.limelight import Limelight
from components.sensors import WheelOfFortuneSensor
from components.serializer import Serializer
#from components.drivetrain import Drivetrain
class Shooter:
    shooter_motor: WPI_TalonSRX
    #color_sensro : WheelOfFortuneSensor
    serializer : Serializer
    limelight : Limelight
    #drivetrain : Drivetrain

    def __init__(self):
        self.power = 0
        self.target_rpm = 0
        self.shooter_kP = 0
        self.shooter_kF = 0
        self.state = ShooterState.SHOOTER_OFF

    def get_raw_velocity(self):
        return self.shooter_motor.getEncoder().getVelocity()

    def update_pid(self, kP, kF):
        self.shooter_kP = kP
        self.shooter_kF = kF

    def set_rpm(self, rpm):
        self.target_rpm = rpm

    def update_motor_velocity(self):
        rpm_error = self.shooter_motor.getEncoder().getVelocity() - self.target_rpm
        kP = self.shooter_kP * rpm_error
        kF = self.target_rpm * self.shooter_kF
        self.power = kP + kF

    def reset(self):
        self.target_rpm = 0
        self.power = 0
    
    def distance_power_calculator(self):
        power = self.limelight.get_distance_trig(RobotMap.Shooter.target_height) * .1
        return power

    def set_power(self, power):
        self.power = power

    def fire(self):
        if(self.state == ShooterState.SHOOTER_OFF):
            self.state = ShooterState.SHOOTER_ON
            self.set_power(self.distance_power_calculator())

    def stop_fire(self):
        if(self.state == ShooterState.SHOOTER_ON):
            self.state = ShooterState.SHOOTER_OFF
            self.serializer.turn_off
            self.set_power(0)
            #self.drivetrain.reset_state()

    def adjust_power(self, rpm):
        self.power += (rpm/200)
        self.power = min(max(-RobotMap.Shooter.max_power,self.power),RobotMap.Shooter.max_power)
        print(self.power)

    def execute(self):

        #self.update_motor_velocity()
        #print('hello')
        self.shooter_motor.set(self.power)
       #if(self.target_rpm > 0 and abs(target_rpm-self.shooter_motor.getEncoder().getVelocity()<100)):
        #    self.serializer.turn_on()


class ShooterState:
    # 0-9 == Shoot
    SHOOTER_ON = 0
    SHOOTER_OFF = 1