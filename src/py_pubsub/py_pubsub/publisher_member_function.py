import rclpy
from rclpy.node import Node

from odrive_can.msg import ControlMessage
from odrive.enums import InputMode
from odrive.enums import ControlMode
from . import gamepad_input
import os


class MinimalPublisher(Node):

    def __init__(self):

        # Give the node a name.
        super().__init__('minimal_publisher')

        # Specify data type and topic name. Specify queue size (limit amount of queued messages)
        self.publisher_axis0 = self.create_publisher(ControlMessage, '/odrive_axis0/control_message', 10)
        self.publisher_axis1 = self.create_publisher(ControlMessage, '/odrive_axis1/control_message', 10)
        self.publisher_axis2 = self.create_publisher(ControlMessage, '/odrive_axis2/control_message', 10)
        self.publisher_axis3 = self.create_publisher(ControlMessage, '/odrive_axis3/control_message', 10)

        # Create a timer that will call the 'timer_callback' function every timer_period second.
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.max_speed = 30

        self.gamepad_deadzone = 0.1

        config_path = os.path.join(os.path.dirname(os.path.realpath(__file__)),
                    '../../../../share/py_pubsub/gamepads.config')

        gamepad_input.setConfigFile(config_path)
        gamepad_input.run_event_loop()
        


    def timer_callback(self):
        
        left_msg = ControlMessage()
        right_msg = ControlMessage()

        gamepad = gamepad_input.getGamepad(0)
        
        if gamepad != None:
            (ls_x, ls_y) = gamepad_input.getLeftStick(gamepad, self.gamepad_deadzone)
            (rs_x, rs_y) = gamepad_input.getRightStick(gamepad, self.gamepad_deadzone)

        
        left_msg.control_mode = ControlMode.VELOCITY_CONTROL
        left_msg.input_mode = InputMode.PASSTHROUGH
        left_msg.input_vel = float(-ls_y * self.max_speed)

        right_msg.control_mode = ControlMode.VELOCITY_CONTROL
        right_msg.input_mode = InputMode.PASSTHROUGH
        right_msg.input_vel = float(-rs_y * self.max_speed)

        # Publish the message.

        # Right side
        self.publisher_axis0.publish(right_msg)
        self.publisher_axis1.publish(right_msg)

        # Left side
        self.publisher_axis2.publish(left_msg)
        self.publisher_axis3.publish(left_msg)

        print("Running")


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
