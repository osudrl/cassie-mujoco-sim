import inputs
import math

EVENT_ABB = (
    # D-PAD, aka HAT
    ('Absolute-ABS_HAT0X', 'HX'),
    ('Absolute-ABS_HAT0Y', 'HY'),

    # Face Buttons
    ('Key-BTN_NORTH', 'N'),
    ('Key-BTN_EAST', 'E'),
    ('Key-BTN_SOUTH', 'S'),
    ('Key-BTN_WEST', 'W'),

    # Other buttons
    ('Key-BTN_THUMBL', 'THL'),
    ('Key-BTN_THUMBR', 'THR'),
    ('Key-BTN_TL', 'TL'),
    ('Key-BTN_TR', 'TR'),
    ('Key-BTN_MODE', 'M'),
    ('Key-BTN_START', 'ST'),
    ('Key-BTN_SELECT', 'SL'),

    ('Absolute-ABS_X', 'LX'),
    ('Absolute-ABS_Y', 'LY'),
    ('Absolute-ABS_RX', 'RX'),
    ('Absolute-ABS_RY', 'RY'),
    ('Absolute-ABS_Z', 'Z'),
    ('Absolute-ABS_RZ', 'RZ')
)

class cassieXboxController:

    def __init__(self, deadzone_ratio = 0.1, abbrevs=EVENT_ABB):
        # Init stuff here

        try:
            self.gamepad = inputs.devices.gamepads[0]
        except IndexError:
            raise inputs.UnpluggedError("No gamepad found.")

        self.deadzone_ratio = deadzone_ratio
        self.analog_state = {}
        self.button_state = {}
        self.abbrevs = dict(abbrevs)
        for key, value in self.abbrevs.items():
            if key.startswith('Absolute'):
                self.analog_state[value] = 0
            if key.startswith('Key'):
                self.button_state[value] = 0

    def process_events(self):
        """Process available events."""
        try:
            events = self.gamepad.read()
        except EOFError:
            events = []
        for event in events:
            self.process_event(event)
        return events
    
    def process_event(self, event):
        """Process the event into a state."""
        if event.ev_type == 'Sync':
            return
        if event.ev_type == 'Misc':
            return
        key = event.ev_type + '-' + event.code
        try:
            abbv = self.abbrevs[key]
        except KeyError:
            return
        
        if event.ev_type == 'Key':
            self.button_state[abbv] = event.state
        if event.ev_type == 'Absolute':
            self.analog_state[abbv] = event.state

    def get_controller_state(self):
        joystick_values = [0,0,0]
        # try:
        flip_side = self.button_state['S']
        joystick_values[0] = self.analog_state['LX']*(1.0/2**15)
        joystick_values[1] = self.analog_state['LY']*(1.0/2**15)
        joystick_values[2] = (self.analog_state['RZ'] - self.analog_state['Z'])*(1.0/2**10)
        for i in range(3):
            if abs(joystick_values[i]) < self.deadzone_ratio:
                joystick_values[i] = 0
            else:
                joystick_values[i] -= math.copysign( self.deadzone_ratio,joystick_values[i])
        return flip_side, joystick_values
        # except:
            # return 0, [0,0,0]
    

def main():
    import time
    """Process all events forever."""
    cntrler = cassieXboxController()
    while 1:
        cntrler.process_events()
        # cntrler.process_all_events()
        flip, vals = cntrler.get_controller_state()
        print("Flip: " + str(flip))
        print(vals)
        time.sleep(1.0/50)




if __name__ == "__main__":
    main()   
