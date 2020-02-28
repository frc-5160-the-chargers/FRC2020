from magicbot import AutonomousStateMachine, timed_state, state

from components.drivetrain import Drivetrain, DrivetrainState

from fieldMeasurements import FieldMeasurements

class DriveLineAuto(AutonomousStateMachine):
    MODE_NAME = "Driveline Auto"
    DEFAULT = True

    drivetrain: Drivetrain

    @state(first=True)
    def drive_off_line(self, initial_call):
        if initial_call:
            self.drivetrain.drive_to_position(FieldMeasurements.DrivelineAuto.driveline_distance)
        elif self.drivetrain.pid_manager.get_on_target():
            self.drivetrain.stop()
            self.done()