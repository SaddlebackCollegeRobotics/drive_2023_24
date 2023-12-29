import os
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from ament_index_python.packages import get_package_share_directory

from . import gamepad_input

# TODO - Add service for changing max speed of motors for drive system.

class DriveInputPublisher(Node):

    def __init__(self):

        # Give the node a name.
        super().__init__('drive_input_publisher')

        self.GAMEPAD_INDEX = 0
        self.GAMEPAD_DEADZONE = 0.1  # 0 to 1
        self.PUBLISHER_PERIOD = 1/10 # seconds

        self.control_publisher = self.create_publisher(Float64MultiArray, '/control/drive_control_input', 10)
        self.msg = Float64MultiArray()

        self.configure_gamepad_input()

        self.timer = self.create_timer(self.PUBLISHER_PERIOD, self.timer_callback)

    def timer_callback(self):
        gamepad = gamepad_input.getGamepad(self.GAMEPAD_INDEX)

        if gamepad is not None:
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
                self.msg.data = [0, 0]
            elif trigger_right or trigger_left:
                self.msg.data = [trigger_right - trigger_left] * 2
                # Rotation logic ...

                # Left turn (left_stick_x: [-1, 0)): decrease speed of left side
                # Right turn (left_stick_x: (0, 1]): decrease speed of right side
                
                if left_stick_x < 0: # Left side
                    if self.msg.data[0] > 0:
                        self.msg.data[0] -= abs(left_stick_x)
                    else:
                        self.msg.data[0] += abs(left_stick_x)
                elif left_stick_x > 0: # Right side
                    if self.msg.data[1] > 0:
                        self.msg.data[1] -= left_stick_x
                    else:
                        self.msg.data[1] += left_stick_x
            else:
                # Point turn logic
                turn_amount = abs(left_stick_x)

                if left_stick_x < 0: # Left turn
                    self.msg.data = [-turn_amount, turn_amount]
                elif left_stick_x > 0: # Right turn
                    self.msg.data = [turn_amount, -turn_amount]
                else:
                    self.msg.data = [0, 0]

        self.msg.data = [round(self.msg.data[0], 3), round(self.msg.data[1], 3)]

        self.control_publisher.publish(self.msg)
            
    def configure_gamepad_input(self):

        config_path = os.path.join(get_package_share_directory('py_pubsub'), 'gamepads.config')
        gamepad_input.setConfigFile(config_path)

        gamepad_input.run_event_loop()


def main(args=None):
    rclpy.init(args=args)

    drive_input_publisher = DriveInputPublisher()

    rclpy.spin(drive_input_publisher)

    # Destroy the node explicitly
    drive_input_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
