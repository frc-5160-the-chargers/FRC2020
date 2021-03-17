from magicbot import AutonomousStateMachine,state
from components.drivetrain import Drivetrain
from components.limelight import Limelight

class LimelightTurnAuto(AutonomousStateMachine):
    drivetrain: Drivetrain
    limelight: Limelight

    MODE_NAME = "Limelight Followy Auto"
    DEFAULT = False



    @state(first=True)
    def follow_target(self,initial_call):
        #print();
        if initial_call:
            self.limelight.switch_pipeline(0);
            self.drivetrain.drive_to_limelight_target(0.5);



