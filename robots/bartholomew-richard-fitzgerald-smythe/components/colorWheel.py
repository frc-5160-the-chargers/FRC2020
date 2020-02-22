from components.sensors import WheelOfFortuneSensor
from magicbot.state_machine import StateMachine,state,timed_state,default_state

class ColorWheelController(StateMachine):
    color_sensor: WheelOfFortuneSensor

    spin_power = 0.5;

    def __init__(self):
        self.current_color = None;
        self.last_color = None;
        self.target_color = None;
        self.motor_power = 0;
        self.manual_power = 0;
        self.target_steps;



    def set_motor_power(self,power):
        self.motor_power = 0;

    @default_state
    def stop(self):
        self.set_motor_power(0);

    @state
    def manual_power(self):
        self.set_motor_power(self.manual_power);

    @state(must_finish=True)
    def spin_distance(self,initial_call=False):
        if (initial_call):
            self.steps_moved = 0;
        direction = 1 if (self.steps_moved - self.target_steps) > 0 else -1;
        if (self.color_sensor.last_color != self.color_sensor.current_color):
            self.steps_moved += direction;
        self.set_motor_power(direction*self.spin_power);
        if (self.steps_moved == self.target_steps):
            self.next_state('stop');

    @state(must_finish=True)
    def spin_to_color(self):
        if (self.color_sensor.current_color == self.target_color):
            self.next_state('stop');
            
    def rotate_by_steps(self,steps):
        self.target_steps = steps;
        self.next_state('spin_distance');

    def rotate_to_color(self,color):
        self.target_color = color;
        self.next_state('spin_to_color');

    def manual_power_input(self,power):
        self.manual_power = power;

    def get_color_value(self):
        return self.color_sensor.get_rgb();

    def get_color_name(self):
        return self.current_color;

