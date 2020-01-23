import wpilib
import wpilib.drive

import magicbot

from rev import CANSparkMax, MotorType

from oi import Driver, Sysop
from utils import config_spark
from robotmap import RobotMap

class Robot(magicbot.MagicRobot):
    def createObjects(self):
        motor_objects_left = [
            CANSparkMax(port, MotorType.kBrushless) for port in RobotMap.Drivetrain.motors_left
        ]
        self.motors_left = wpilib.SpeedControllerGroup(motor_objects_left[0], *motor_objects_left[1:])

        motor_objects_right = [
            CANSparkMax(port, MotorType.kBrushless) for port in RobotMap.Drivetrain.motors_right
        ]
        self.motors_right = wpilib.SpeedControllerGroup(motor_objects_right[0], *motor_objects_right[1:])

        for motor in motor_objects_left + motor_objects_right:
            config_spark(motor, RobotMap.Drivetrain.motor_config)

        self.drivetrain = wpilib.drive.DifferentialDrive(self.motors_left, self.motors_right)
        self.drivetrain.setMaxOutput(RobotMap.Drivetrain.max_motor_power)

        self.driver = Driver(wpilib.XboxController(0))
        self.sysop = Sysop(wpilib.XboxController(1))

    def reset_subsystems(self):
        pass

    def teleopInit(self):
        self.reset_subsystems()

    def teleopPeriodic(self):
        x, y = self.driver.get_curvature_output()
        self.drivetrain.curvatureDrive(y, x, True)

if __name__ == '__main__':
    wpilib.run(Robot)