from magicbot import AutonomousStateMachine, state
from magicbot.state_machine import timed_state

from components.drivetrain import Drivetrain
from components.intake import Intake, IntakeLift, IntakeRoller, IntakeRollerState
from components.sensors import NavX
from components.limelight import Limelight
from utils import AngleUtils


class GalacticPaths:
    BLUE_PATH = 0;
    RED_PATH = 1;
    
    PATHS = [
        [0,21.1,-62.0,14],
        [0,21.1,-62.0,14],
        [0,0,0,0],
        [0,0,0,0]
    ]

    ANGLE_TOLERANCE = 10;

    CELL_DISTANCE = 0.2; #meters

    CELL_IMAGE_MARGIN = 8; #degrees
    
    PICK_UP_DISTANCE = 40; #inches

    LIFT_POWER = 0.5;

class GalacticAuto(AutonomousStateMachine):

    MODE_NAME = "Galactic Search Auto"
    DEFAULT = False

    drivetrain: Drivetrain;
    limelight: Limelight;
    intake_lift: IntakeLift;
    intake_roller: IntakeRoller;
    navx: NavX

    @state(first=True)
    def start_search(self,initial_call):
        print("autonomous initialized");
        self.collected_cells = 0; #updates when power cell has been collected into intake
        self.path = -1;
        self.limelight.switch_pipeline(3);
        self.navx.reset();
        #self.lift.lower_lift(0.5);
        self.next_state_now('find_power_cell');
        


    @state() #should be called three times; gets power cell into frame
    def find_power_cell(self,initial_call):
        print("searching for power cell",self.collected_cells+1,"on path",self.path,"turning towards angle",GalacticPaths.PATHS[self.path][self.collected_cells],"with error",self.drivetrain.turn_pid.get_error(),";",("ball estimated at global angle" + str(self.navx.get_heading() + self.limelight.horizontal_offset)) if self.limelight.valid_target else "");
        if self.limelight.valid_target:
            if self.collected_cells == 0: #identify path; assuming that both front cells should be visible from start location - might need change based on testing
                print("Initial target spotted at angle",self.navx.get_heading() + self.limelight.horizontal_offset);
                self.path = AngleUtils.closest_angle(self.navx.get_heading() + self.limelight.horizontal_offset,GalacticPaths.PATHS[GalacticPaths.BLUE_PATH][0],GalacticPaths.PATHS[GalacticPaths.RED_PATH][0]);
                self.next_state('target_power_cell');
            elif (abs(self.limelight.horizontal_offset) < self.limelight.FOV/2-GalacticPaths.CELL_IMAGE_MARGIN and abs(self.navx.get_heading() + self.limelight.horizontal_offset - GalacticPaths.PATHS[self.path][self.collected_cells]) < GalacticPaths.ANGLE_TOLERANCE):
                print("Target Spotted");
                self.next_state('target_power_cell');
        
        if initial_call:
            if self.path != -1:
                self.drivetrain.turn_to_angle(GalacticPaths.PATHS[self.path][self.collected_cells]);


    @state() #point towards the limelight target
    def target_power_cell(self,initial_call):
        print("Centering on cell; angle:",self.limelight.get_horizontal_angle_offset());
        if initial_call:
            self.drivetrain.turn_to_limelight_target();
        if self.drivetrain.limelight_turn_pid.get_on_target():
            self.next_state("drive_to_power_cell");


    @state()
    def drive_to_power_cell(self,initial_call):
        print("Full Speed Ahead; distance:",self.limelight.get_distance_trig(0.12));
        if initial_call:
            self.drivetrain.drive_to_limelight_target(GalacticPaths.CELL_DISTANCE,0.12);
        if self.drivetrain.limelight_distance_pid.get_on_target():
            self.next_state("pick_up_power_cell");
        
    @state()
    def pick_up_power_cell(self,initial_call):
        if initial_call:
            self.drivetrain.drive_to_position(GalacticPaths.PICK_UP_DISTANCE);
        if self.intake_roller.state != IntakeRollerState.INTAKING and self.intake_lift.get_ready():
            self.intake_roller.intake();
        if self.drivetrain.position_pid.get_on_target():
            self.collected_cells += 1;
            if self.collected_cells >= 3:
                #self.next_state("find_end");
                self.next_state("purgatory");
            else:
                self.next_state("wait_for_power_cell_removal");

    @state()
    def purgatory(self):
        pass;

    @timed_state(duration=10,next_state="find_power_cell")
    def wait_for_power_cell_removal(self):
        
        self.intake_roller.stop();
        print("path",self.path);
        pass;



    @state()
    def find_end(self,initial_call): #assuming limelight-based end recognition; going to go directly to limelight target
        if initial_call:
            self.limelight.switch_pipeline(2);
        if self.limelight.valid_target and (abs(self.navx.get_heading() + self.limelight.horizontal_offset - GalacticPaths.PATHS[self.path][self.collected_cells]) < GalacticPaths.ANGLE_TOLERANCE):
            print("End Spotted");
            self.next_state('drive_to_end');
        elif initial_call:
            self.drivetrain.turn_to_angle(GalacticPaths.PATHS[self.path][self.collected_cells]);            

    @state()
    def drive_to_end(self,initial_call):
        if initial_call:
            self.drivetrain.drive

            



