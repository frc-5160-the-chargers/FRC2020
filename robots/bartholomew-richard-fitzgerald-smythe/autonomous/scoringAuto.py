from magicbot import AutonomousStateMachine, state, timed_state

from components.drivetrain import Drivetrain, DrivetrainState
from components.intake import Intake

from fieldMeasurements import FieldMeasurements

class ScoringAuto(AutonomousStateMachine):
    # this auto will drive up against the scoring port and outtake
    MODE_NAME = "Basic Scoring Auto"
    DEFAULT = True

    drivetrain: Drivetrain
    intake: Intake
    
    @state(first=True)
    def lower_lift(self, initial_call):
        if initial_call:
            self.intake.reset()
            self.intake.intake_lift.send_down()
        elif self.intake.intake_lift.pid_controller.get_on_target():
            self.next_state('drive_to_scoring')
    
    @timed_state(duration=7, next_state='outtake')
    def drive_to_scoring(self, initial_call):
        if initial_call:
            self.drivetrain.drive_to_position(FieldMeasurements.ScoringAuto.drive_distance)
            self.intake.intake_lift.send_up()
        elif self.drivetrain.pid_manager.get_on_target():
            self.drivetrain.stop()
            self.next_state('outtake')

    @timed_state(duration=FieldMeasurements.ScoringAuto.outtake_time, next_state='stop_outtake')
    def outtake(self):
        self.intake.intake_roller.intake()

    @state()
    def stop_outtake(self):
        self.intake.intake_roller.stop()