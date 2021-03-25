from components.drivetrain import Powertrain
from magicbot import AutonomousStateMachine,state
from components.drivetrain import Drivetrain
from components.limelight import Limelight
from wpilib import SmartDashboard

class LimelightTurnAuto(AutonomousStateMachine):
    drivetrain: Drivetrain
    limelight: Limelight
    powertrain: Powertrain

    MODE_NAME = "Drive Forward Auto"
    DEFAULT = False



    @state(first=True)
    def drive_forward(self,initial_call):
        #print();
        if initial_call:
            self.drivetrain.drive_to_position(30);
        print(self.drivetrain.position_pid.get_error())
        #SmartDashboard.putNumber("power: ",self.powertrain.power);
        #SmartDashboard.putNumber("rotation: ",self.powertrain.rotation);



