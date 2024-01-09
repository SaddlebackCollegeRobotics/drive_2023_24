import os
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from .controller_manager import ControllerScheme, ControllerManager

# TODO - Add service for changing max speed of motors for drive system.

class DriveInputPublisher(Node):

    def __init__(self):

        # Give the node a name.
        super().__init__('drive_input_publisher')

        self.PUBLISHER_PERIOD = 1/10 # seconds

        self.control_publisher = self.create_publisher(Float64MultiArray, '/control/drive_control_input', 10)
        self.msg = Float64MultiArray()

        self._controller_manager = ControllerManager(ControllerScheme.BASIC)

        self.timer = self.create_timer(self.PUBLISHER_PERIOD, self.timer_callback)

    def timer_callback(self):
        
        move_vec = self._controller_manager.handle_input()

        self.msg.data = move_vec
        self.control_publisher.publish(self.msg)


def main(args=None):
    rclpy.init(args=args)

    drive_input_publisher = DriveInputPublisher()

    rclpy.spin(drive_input_publisher)

    # Destroy the node explicitly
    drive_input_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
