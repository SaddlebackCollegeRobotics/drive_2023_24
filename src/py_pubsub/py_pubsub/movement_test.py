import os

import gamepad_input

def setup():
    config_path = os.path.join('../config/gamepads.config')
    gamepad_input.setConfigFile(config_path)
    gamepad_input.run_event_loop()

def main():
    setup()

    data = [0, 0]
    while True:
        gamepad = gamepad_input.getGamepad(0)

        (left_stick_x, _) = gamepad_input.getLeftStick(gamepad, 0.1)
        (trigger_left, trigger_right) = gamepad_input.getTriggers(gamepad, 0.1)

        # TODO: Better logic!!!
        
        # TODO: Safety shutoff
        # TODO: Cruise control
        # TODO: Rumbling controller for rough terrain
        """ TODO: Main controlling
                - Left trigger = backwards
                - Right trigger = forward
                - Left joystick (x-axis) = turning
        """
        
        # D
        if trigger_right and trigger_left:
            data = [0, 0]
        elif trigger_right or trigger_left:
            data = [trigger_right - trigger_left] * 2
            # Rotation logic ...

            # Left turn (left_stick_x: [-1, 0)): decrease speed of left side
            # Right turn (left_stick_x: (0, 1]): decrease speed of right side
            
            if left_stick_x < 0: # Left side
                if data[0] > 0:
                    data[0] -= abs(left_stick_x)
                else:
                    data[0] += abs(left_stick_x)
            elif left_stick_x > 0: # Right side
                if data[1] > 0:
                    data[1] -= left_stick_x
                else:
                    data[1] += left_stick_x
        else:
            # Point turn logic
            turn_amount = abs(left_stick_x)

            if left_stick_x < 0: # Left turn
                data = [-turn_amount, turn_amount]
            elif left_stick_x > 0: # Right turn
                data = [turn_amount, -turn_amount]
            else:
                data = [0, 0]
        data = [round(data[0], 3), round(data[1], 3)]

if __name__ == '__main__':
    main()