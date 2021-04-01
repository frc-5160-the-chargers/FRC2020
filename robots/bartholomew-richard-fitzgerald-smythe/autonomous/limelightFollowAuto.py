from components.drivetrain import Powertrain
from magicbot import AutonomousStateMachine,state
from components.drivetrain import Drivetrain
from components.limelight import Limelight
from wpilib import SmartDashboard

class LimelightTurnAuto(AutonomousStateMachine):
    drivetrain: Drivetrain
    limelight: Limelight
    powertrain: Powertrain

    MODE_NAME = "Limelight Followy Auto"
    DEFAULT = False



    @state(first=True)
    def follow_target(self,initial_call):
        #print();
        if initial_call:
            self.limelight.switch_pipeline(0);
            self.drivetrain.drive_to_limelight_target(0.2,0.15);
        #print(self.drivetrain.limelight_distance_pid.get_error())
        SmartDashboard.putNumber("power: ",self.powertrain.power);
        SmartDashboard.putNumber("rotation: ",self.powertrain.rotation);



