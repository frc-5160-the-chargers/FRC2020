from magicbot import AutonomousStateMachine, state

from ..components.drivetrain import Drivetrain
from ..components.intake import Intake, IntakeLift, IntakeRoller, IntakeRollerState
from ..components.sensors import NavX
from ..components.limelight import Limelight
from ..utils import AngleUtils


class GalacticPaths:
    BLUE_PATH = 0;
    RED_PATH = 1;
    
    PATHS = [
        [0,0,0,0],
        [0,0,0,0],
        [0,0,0,0],
        [0,0,0,0]
    ]

    ANGLE_TOLERANCE = 10;

    CELL_DISTANCE = 10;

    LIFT_POWER = 0.5

class GalacticAuto(AutonomousStateMachine):

    drivetrain: Drivetrain;
    limelight: Limelight;
    lift: IntakeLift;
    intake: IntakeRoller;
    navx: NavX

    @state(first=True)
    def start_search(self):
        self.collected_cells = 0; #updates when power cell has been collected into intake
        self.path = -1;
        self.limelight.switch_pipeline(1);
        self.navx.reset();
        self.next_state_now('find_power_cell');
        self.lift.lower_lift(0.5);


    @state() #should be called three times; gets power cell into frame
    def find_power_cell(self,initial_call):
        if self.limelight.valid_target():
            if self.collected_cells == 0: #identify path; assuming that both front cells should be visible from start location - might need change based on testing
                print("Initial target spotted");
                self.path = AngleUtils.closest_angle(self.navx.get_heading() + self.limelight.horizontal_offset,GalacticPaths.BLUE_ANGLES[0],GalacticPaths.RED_ANGLES[0]);
                self.next_state('target_power_cell');
            elif (abs(self.navx.get_heading() + self.limelight.horizontal_offset - GalacticPaths.PATHS[self.path][self.collected_cells]) < GalacticPaths.ANGLE_TOLERANCE):
                print("Target Spotted");
                self.next_state('target_power_cell');
        if initial_call:
            if self.path != -1:
                self.drivetrain.turn_to_angle(GalacticPaths.PATHS[self.path][self.collected_cells]);


    @state() #point towards the limelight target
    def target_power_cell(self,initial_call):
        if initial_call:
            self.drivetrain.turn_to_limelight_target();
        if self.drivetrain.limelight_turn_pid.get_on_target():
            self.next_state("drive_to_power_cell");


    @state()
    def drive_to_power_cell(self,initial_call):
        if initial_call:
            self.drivetrain.drive_to_limelight_target(GalacticPaths.CELL_DISTANCE);
        if self.intake.state != IntakeRollerState.INTAKING and self.lift.get_ready():
            self.intake.intake();
        if self.drivetrain.limelight_distance_pid.get_on_target():
            self.collected_cells += 1;
            if self.collected_cells == 3:
                self.next_state("find_end");
            else:
                self.next_state("find_power_cell");

    @state()
    def find_end(self,initial_call): #assuming limelight-based end recognition; going to go directly to limelight target
        if initial_call:
            self.limelight.switch_pipeline(2);
        if self.limelight.valid_target() and (abs(self.navx.get_heading() + self.limelight.horizontal_offset - GalacticPaths.PATHS[self.path][self.collected_cells]) < GalacticPaths.ANGLE_TOLERANCE):
            print("End Spotted");
            self.next_state('drive_to_end');
        elif initial_call:
            self.drivetrain.turn_to_angle(GalacticPaths.PATHS[self.path][self.collected_cells]);            

    @state()
    def drive_to_end(self,initial_call):
        if initial_call:
            self.drivetrain.drive

            



