from os import path
from time import sleep
from enum import Enum
from functools import partial
# Allow for this program to run standalone for testing without ROS packages
if __name__ != '__main__':
    from ament_index_python.packages import get_package_share_directory
    from . import gamepad_input as gi
else:
    import gamepad_input as gi

__all__ = [
    'ControllerScheme',
    'ControllerManager',
]

class ControllerScheme(Enum):
    """Set of valid control schemas with their logic.
    
    Each scheme is a function accepting an input state and returning a vector
    to send to the rover.

    input_state: (ls_x, ls_y, rs_x, rs_y, lt, rt, hat_x, hat_y, a, b, x, y)

    Plus, minus, and home buttons are currently reserved for general logic
    """

    @staticmethod
    def _basic(input_state) -> (float, float):
        """Basic input scheme internal logic
        
        Left stick y controls left motors, same for right stick respectively
        Left and right triggers make in-place turns
        Stick controls take priority; trigger logic is disabled when sticks are
        active
        """
        _, ls_y, _, rs_y, lt, rt, *_ = input_state

        move_vec = (ls_y, rs_y)

        # Check that: neither stick is active AND exactly one trigger is active
        if not (ls_y or rs_y) and (bool(lt) != bool(rt)):
            if lt:
                move_vec = (-lt, lt)
            elif rt:
                move_vec = (rt, -rt)

        return move_vec

    @staticmethod
    def _trigger_based(input_state) -> (float, float):
        """Trigger based input scheme internal logic"""
        # TODO: Refactor logic!
        ls_x, _, _, _, lt, rt, *_ = input_state

        if rt and lt:
            move_vec = [0.0, 0.0]
        elif rt or lt:
            move_vec = [rt - lt] * 2
            # Rotation logic ...

            # Left turn (ls_x: [-1, 0)): decrease speed of left side
            # Right turn (ls_x: (0, 1]): decrease speed of right side
            
            if ls_x < 0: # Left side
                if move_vec[0] > 0:
                    move_vec[0] -= abs(ls_x)
                else:
                    move_vec[0] += abs(ls_x)
            elif ls_x > 0: # Right side
                if move_vec[1] > 0:
                    move_vec[1] -= ls_x
                else:
                    move_vec[1] += ls_x
        else:
            # Point turn logic
            turn_amount = abs(ls_x)

            if ls_x < 0: # Left turn
                move_vec = [-turn_amount, turn_amount]
            elif ls_x > 0: # Right turn
                move_vec = [turn_amount, -turn_amount]
            else:
                move_vec = [0.0, 0.0]
        
        return tuple(move_vec)


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
    _cruise_vec: (float, float)

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

    def handle_input(self) -> (float, float):
        # Reinitialize gamepad each call to handle new connections
        gamepad = gi.getGamepad(self._gamepad_index)

        if not gamepad:
            print(f'WARN: No valid gamepad at {self._gamepad_index}!')
            return (0.0, 0.0)
    
        ls_x, ls_y = gi.getLeftStick(gamepad, self._deadzone)
        rs_x, rs_y = gi.getRightStick(gamepad, self._deadzone)
        lt, rt = gi.getTriggers(gamepad, self._deadzone)
        hat_x, hat_y = gi.getHat(gamepad)
        # TODO: Verify these button index values
        plus, minus, home = gi.getButtonsValues(10, 11, 12)
        y, x, a, b = gi.getButtonsValues(4, 3, 0, 1)
        
        # Emergency stop: press plus + minus to toggle
        if plus and minus:
            self._stopped = not self._stopped
            sleep(1.0) # HACK: refactor to not allow repeat reads of buttons
        
        if self._stopped:
            return (0.0, 0.0)

        move_vec = self._scheme(ls_x, ls_y, rs_x, rs_y, lt, rt, hat_x, hat_y, \
                                a, b, x, y)

        # Cruise control: press home to toggle 'cruise control'
        if home:
            self._cruise_vec = move_vec if not self._cruise_vec else None
            sleep(1.0) # HACK: refactor to not allow repeat reads of buttons
        
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
            config_path = \
            path.join(get_package_share_directory('py_pubsub'), \
                      'gamepads.config')
        gi.setConfigFile(config_path)

        gi.run_event_loop()


# For testing purposes
if __name__ == '__main__':
    import sys
    manager = ControllerManager(config_path = '../config/gamepads.config')
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
        print(f'{manager.handle_input()}')
        sleep(0.2)

