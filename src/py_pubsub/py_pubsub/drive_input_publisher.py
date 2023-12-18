import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from ament_index_python.packages import get_package_share_directory

from . import gamepad_input


class DriveInputPublisher(Node):

    def __init__(self):

        # Give the node a name.
        super().__init__('drive_input_publisher')

        self.GAMEPAD_INDEX = 0
        self.GAMEPAD_DEADZONE = 0.1  # 0 to 1
        self.PUBLISHER_PERIOD = 1/10 # seconds

        self.control_publisher = self.create_publisher(Float64MultiArray, '/control/drive_input', 10)
        self.msg = Float64MultiArray()

        self.configure_gamepad_input()

        self.timer = self.create_timer(self.PUBLISHER_PERIOD, self.timer_callback)

    def timer_callback(self):

        gamepad = gamepad_input.getGamepad(self.GAMEPAD_INDEX)

        if gamepad != None:
            (ls_x, ls_y) = gamepad_input.getLeftStick(gamepad, self.GAMEPAD_DEADZONE)
            (rs_x, rs_y) = gamepad_input.getRightStick(gamepad, self.GAMEPAD_DEADZONE)
            (l2, r2) = gamepad_input.getTriggers(gamepad, self.GAMEPAD_DEADZONE)

            self.msg.data = [ls_x, ls_y, rs_x, rs_y, l2, r2]

            self.control_publisher.publish(self.msg)
            
    def configure_gamepad_input(self):

        config_path = os.path.join(get_package_share_directory('py_pubsub'), 'gamepads.config')
        gamepad_input.setConfigFile(config_path)

        gamepad_input.run_event_loop()

    # TODO - Add service for changing max speed of motors for drive system.

    


def main(args=None):
    rclpy.init(args=args)

    drive_input_publisher = DriveInputPublisher()

    rclpy.spin(drive_input_publisher)

    # Destroy the node explicitly
    drive_input_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
