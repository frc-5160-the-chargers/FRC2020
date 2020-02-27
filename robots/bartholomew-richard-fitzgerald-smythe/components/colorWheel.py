from components.sensors import WheelOfFortuneSensor
from magicbot.state_machine import StateMachine,state,timed_state,default_state
from ctre import WPI_TalonSRX

class ColorWheelController(StateMachine):
    color_sensor: WheelOfFortuneSensor

    spin_power = 0.5
    manual_slowdown = 0.6

    fortune_motor : WPI_TalonSRX

    def __init__(self):
        self.current_color = None
        self.last_color = None
        self.target_color = None
        self.manual_power = 0
        self.target_steps = 0


    def get_auto_activated(self):
        return self.current_state in ['spin_distance','spin_to_color']

    def set_motor_power(self,power):
        self.fortune_motor.set(power)

    @default_state
    def stop(self):
        self.set_motor_power(0)

    @state
    def manual_power(self):
        self.set_motor_power(self.manual_power*self.manual_slowdown)

    @state(must_finish=True)
    def spin_distance(self,initial_call=False):
        if (initial_call):
            self.steps_moved = 0
        direction = 1 if (self.steps_moved - self.target_steps) > 0 else -1
        if (self.color_sensor.last_color != self.color_sensor.current_color):
            self.steps_moved += direction
        self.set_motor_power(direction*self.spin_power)
        if (self.steps_moved == self.target_steps):
            self.next_state('stop')

    @state(must_finish=True)
    def spin_to_color(self):
        self.set_motor_power(self.spin_power)
        if (self.color_sensor.current_color == self.target_color):
            self.next_state('stop')
        
            
    def rotate_by_steps(self,steps,set_override=False):
        if (self.current_state == 'spin_distance' and not set_override):
            self.target_steps += steps
        else:
            self.target_steps = steps
        self.next_state('spin_distance')

    def rotate_to_color(self,color):
        self.target_color = color
        self.next_state('spin_to_color')

    def manual_power_input(self,power):
        self.manual_power = power
        if (self.current_state != 'manual_power'):
            self.next_state('manual_power')

    def get_color_value(self):
        return self.color_sensor.get_rgb()

    def get_color_name(self):
        return self.current_color

    def deactivate(self):
        self.next_state('stop')