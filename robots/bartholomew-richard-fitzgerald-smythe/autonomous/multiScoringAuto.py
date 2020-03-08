from magicbot import AutonomousStateMachine, state, timed_state

from components.drivetrain import Drivetrain, DrivetrainState
from components.intake import Intake

from fieldMeasurements import FieldMeasurements

class MultiScoringAuto(AutonomousStateMachine):
    # this auto will drive up against the scoring port and outtake
    MODE_NAME = "Multi Scoring Auto"
    DEFAULT = False

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
        self.next_state('back_up_pre_turn')

    @timed_state(duration=3, next_state='stop_robot')
    def back_up_pre_turn(self, initial_call):
        if initial_call:
            self.drivetrain.drive_to_position(-FieldMeasurements.MultiScoringAuto.backup_distance)
        elif self.drivetrain.pid_manager.get_on_target():
            self.drivetrain.stop()
            self.next_state('turn_to_more')

    @timed_state(duration=3, next_state='stop_robot')
    def turn_to_more(self, initial_call):
        if initial_call:
            self.drivetrain.turn_to_angle(FieldMeasurements.MultiScoringAuto.turn_degrees)
        elif self.drivetrain.pid_manager.get_on_target():
            self.drivetrain.stop()
            self.next_state('drive_out')

    @timed_state(duration=3, next_state='stop_robot')
    def drive_out(self, initial_call):
        if initial_call:
            self.drivetrain.drive_to_position(FieldMeasurements.MultiScoringAuto.drive_out_distance)
        elif self.drivetrain.pid_manager.get_on_target():
            self.drivetrain.stop()
            self.next_state('turn_to_more_again')

    @timed_state(duration=3, next_state='stop_robot')
    def turn_to_more_again(self, initial_call):
        if initial_call:
            self.drivetrain.turn_to_angle(FieldMeasurements.MultiScoringAuto.turn_degrees_final_alignment)
        elif self.drivetrain.pid_manager.get_on_target():
            self.drivetrain.stop()
            self.next_state('stop_robot')

    @state()
    def stop_robot(self):
        self.drivetrain.stop()
        self.intake.intake_roller.stop()