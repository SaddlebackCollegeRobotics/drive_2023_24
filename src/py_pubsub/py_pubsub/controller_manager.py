from pathlib import Path
from enum import Enum
from functools import partial
from numpy import clip
# Allow for this program to run standalone for testing without ROS packages
if __name__ != '__main__':
    from ament_index_python.packages import get_package_share_directory
    from . import gamepad_input as gi
    DEBUG = True
else:
    DEBUG = True
    import gamepad_input as gi

class ControllerScheme(Enum):
    """Set of valid control schemas with their logic.
    
    Each scheme is a function accepting an input state and returning a vector
    to send to the rover.

    input_state: (ls_x, ls_y, rs_x, rs_y, lt, rt, hat_x, hat_y, a, b, x, y)

    Plus, minus, and home buttons are currently reserved for general logic
    """

    @staticmethod
    def _basic(*input_state) -> list[float]:
        """Basic input scheme internal logic
        
        Left stick y controls left motors, same for right stick respectively
        Left and right triggers make in-place turns
        Stick controls take priority; trigger logic is disabled when sticks are
        active
        """
        _, ls_y, _, rs_y, lt, rt, *_ = input_state

        move_vec = [ls_y, rs_y]

        # Check that: neither stick is active AND exactly one trigger is active
        if not (ls_y or rs_y) and (bool(lt) != bool(rt)):
            if lt:
                move_vec = [-lt, lt]
            elif rt:
                move_vec = [rt, -rt]

        return move_vec

    @staticmethod
    def _trigger_based(*input_state) -> list[float]:
        """Trigger based input scheme internal logic
        
        Right and left Triggers control forward and backward movement 
        respectively. Left stick x-axis used to a) make in-place turns if used 
        alone, b) turn forward/backward when used in conjunction with triggers.
        If both triggers are pressed at the same time, output is nullified.
        """
        ls_x, _, _, _, lt, rt, *_ = input_state

        move_vec = [0.0, 0.0]
            
        if not (rt and lt) and (rt or lt):
            # Linear movement
            move_vec = [rt - lt] * 2
        
        # Turning
        move_vec[0] -= ls_x
        move_vec[1] += ls_x
        
        # Ensure that values end up [-1, 1]
        move_vec = list(clip(move_vec, -1, 1))

        return move_vec


    BASIC = partial(_basic)
    TRIGGER_BASED = partial(_trigger_based)

    def __call__(self, *args, **kwargs):
        """Allows for treating enumerated values as functions"""
        return self.value(*args, **kwargs)



class ControllerManager:
    """{DOCSTRING HERE}"""
    _scheme: ControllerScheme
    _gamepad_index: int
    _deadzone: float

    _stopped: bool
    _stop_pressed: bool
    _cruise_vec: (float, float)
    _cruise_pressed: bool

    def __init__(self, \
                 scheme: ControllerScheme = ControllerScheme.BASIC, \
                 gamepad_index: int = 0, \
                 deadzone: float = 0.1,
                 config_path: str = None) -> None:
        self._scheme = scheme

        self._configure_gamepad_input(config_path)
        self._gamepad_index = gamepad_index
        self._deadzone = deadzone

        self._stopped = False
        self._stop_pressed = False
        self._cruise_vec = None
        self._cruise_pressed = False

    def handle_input(self) -> list[float]:
        # Reinitialize gamepad each call to handle new connections
        gamepad = gi.getGamepad(self._gamepad_index)

        if not gamepad:
            print(f'WARN: No valid gamepad at {self._gamepad_index}!')
            return [0.0, 0.0]
    
        ls_x, ls_y = gi.getLeftStick(gamepad, self._deadzone)
        rs_x, rs_y = gi.getRightStick(gamepad, self._deadzone)
        # Negate all joystick values to obtain normal results
        ls_x, ls_y, rs_x, rs_y = -ls_x, -ls_y, -rs_x, -rs_y
        lt, rt = gi.getTriggers(gamepad, self._deadzone)
        hat_x, hat_y = gi.getHat(gamepad)

        plus, minus, home = gi.getButtonsValues(gamepad, 4, 5, 6)
        y, x, a, b = gi.getButtonsValues(gamepad, 0, 1, 2, 3)
        ls_b, rs_b = gi.getButtonsValues(gamepad, 9, 10)

        if DEBUG:
            print([i for i,b in \
               enumerate(gi.getButtonsValues(gamepad, *range(0, 18))) if b])

        # Emergency stop: press plus + minus to toggle
        if plus and minus:
            if not self._stop_pressed:
                self._stopped = not self._stopped
                self._stop_pressed = True
        elif self._stop_pressed:
            self._stop_pressed = False
        
        if self._stopped:
            self._cruise_vec = None
            return [0.0, 0.0]

        move_vec = self._scheme(ls_x, ls_y, rs_x, rs_y, lt, rt, hat_x, hat_y, \
                                a, b, x, y)
        
        # Cruise control: press home to toggle
        if home:
            if not self._cruise_pressed:
                self._cruise_vec = move_vec if not self._cruise_vec else None
                self._cruise_pressed = True
        elif self._cruise_pressed:
            self._cruise_pressed = False
        
        if self._cruise_vec:
            return self._cruise_vec
        
        return move_vec


    def change_scheme(self, new_scheme: ControllerScheme) -> None:
        if new_scheme == self._scheme:
            print("WARN: Attempted to change controller scheme to current!")
        else:
            self._scheme = new_scheme

    def _configure_gamepad_input(self, config_path: str) -> None:
        if not config_path:
            config_path = 'gamepads.config'
        config_path_str = str((Path(__file__).parent / config_path).resolve())
        
        gi.setConfigFile(config_path_str)
        gi.run_event_loop()


# For testing purposes
if __name__ == '__main__':
    from time import sleep
    import os
    manager = ControllerManager(config_path = \
                                'src/py_pubsub/config/gamepads.config')
    schemes_list = list(map(lambda x: x.name.lower(), ControllerScheme))
    test_scheme = input(f'Enter an input scheme to test {schemes_list}: ')
    if test_scheme.lower() in schemes_list:
        print(f'Testing {test_scheme} control scheme.')
    else:
        print('Error, invalid scheme name!')
        exit()

    manager.change_scheme(ControllerScheme[test_scheme.upper()])

    sleep(3)

    while True:
        os.system('cls||clear')
        print(f'{manager.handle_input()}')
        sleep(0.2)

