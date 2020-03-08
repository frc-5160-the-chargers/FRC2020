from magicbot import AutonomousStateMachine, timed_state, state

from components.drivetrain import Drivetrain, DrivetrainState
from components.intake import Intake

from fieldMeasurements import FieldMeasurements

class DriveLineAuto(AutonomousStateMachine):
    MODE_NAME = "Driveline Auto"
    DEFAULT = False

    drivetrain: Drivetrain
    intake: Intake

    @state(first=True)
    def drive_off_line(self, initial_call):
        if initial_call:
            self.drivetrain.drive_to_position(FieldMeasurements.DrivelineAuto.driveline_distance)
            self.intake.reset()
            self.intake.intake_lift.set_match_start()
        elif self.drivetrain.pid_manager.get_on_target():
            self.drivetrain.stop()
            self.done()