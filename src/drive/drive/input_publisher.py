import os
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Bool
from std_srvs.srv import Empty

from .controller_manager import ControllerScheme, ControllerManager

# TODO - Add service for changing max speed of motors for drive system.

class DriveInputPublisher(Node):

    def __init__(self):

        # Give the node a name.
        super().__init__('drive_input_publisher')

        self.PUBLISHER_PERIOD = 1/10 # seconds

        self.control_publisher = self.create_publisher(Float64MultiArray, '/drive/control_input', 10)
        self.reset_drive_cli = self.create_client(Empty, '/drive/reset_system')
        
        self.msg = Float64MultiArray()
        self.reset_drive_request = Empty.Request()

        self._controller_manager = ControllerManager(ControllerScheme.TRIGGER_BASED)

        self.timer = self.create_timer(self.PUBLISHER_PERIOD, self.timer_callback)

    def timer_callback(self):
        move_vec = self._controller_manager.handle_input()

        if isinstance(move_vec, list):
            self.msg.data = move_vec
            self.control_publisher.publish(self.msg)
        else:
            self.reset_drive()

    def reset_drive(self):
        
        if (self.reset_drive_cli.service_is_ready() == False):
            print("Warning: Drive reset service is unavailable!")
            return
        
        self.future = self.reset_drive_cli.call_async(self.reset_drive_request)
        self.future.add_done_callback(self.reset_drive_callback)        
        
    def reset_drive_callback(self, future):
        if (future.result() != None):
            print("Successfully reset drive system!")
        else:
            print("Warning: Failed to reset drive system!")

def main(args=None):
    rclpy.init(args=args)

    drive_input_publisher = DriveInputPublisher()

    rclpy.spin(drive_input_publisher)

    # Destroy the node explicitly
    drive_input_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
