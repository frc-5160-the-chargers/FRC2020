# from magicbot import AutonomousStateMachine, timed_state, state

# from components.drivetrain import Drivetrain, DrivetrainState
# from components.intake import Intake

# from fieldMeasurements import FieldMeasurements

# class PushBotAuto(AutonomousStateMachine):
#     # this auto is intended to push other robots off their lines
#     MODE_NAME = "PushBot Auto"
#     DEFAULT = False

#     drivetrain: Drivetrain
#     intake: Intake

#     @state(first=True)
#     def drive_towards_stations(self, initial_call):
#         if initial_call:
#             self.drivetrain.drive_to_position(FieldMeasurements.PushBotAuto.initial_drive_distance)
#             self.intake.reset()
#             self.intake.intake_lift.set_match_start()
#         elif self.drivetrain.pid_manager.get_on_target():
#             self.drivetrain.stop()
#             self.next_state('turn_towards_robot')

#     @state()
#     def turn_towards_robot(self, initial_call):
#         if initial_call:
#             self.drivetrain.turn_to_angle(-90)
#         elif self.drivetrain.pid_manager.get_on_target():
#             self.drivetrain.stop()
#             self.next_state('drive_towards_robot')
    
#     @state()
#     def drive_towards_robot(self, initial_call):
#         if initial_call:
#             self.drivetrain.drive_to_position(FieldMeasurements.PushBotAuto.distance_to_bot)
#         elif self.drivetrain.pid_manager.get_on_target():
#             self.drivetrain.stop()
#             self.next_state('turn_pre_push_bot')
    
#     @state()
#     def turn_pre_push_bot(self, initial_call):
#         if initial_call:
#             self.drivetrain.turn_to_angle(-90)
#         elif self.drivetrain.pid_manager.get_on_target():
#             self.drivetrain.stop()
#             self.next_state('push_bot')

#     @state()
#     def push_bot(self, initial_call):
#         if initial_call:
#             self.drivetrain.drive_to_position(
#                 FieldMeasurements.PushBotAuto.distance_to_bot
#                 + FieldMeasurements.PushBotAuto.extra_distance
#             )
#         elif self.drivetrain.pid_manager.get_on_target():
#             self.drivetrain.stop()
#             self.done()