import rclpy
from rclpy.node import Node

from odrive_can.msg import ControlMessage
from odrive.enums import InputMode
from odrive.enums import ControlMode

class MinimalPublisher(Node):

    def __init__(self):

        # Give the node a name.
        super().__init__('minimal_publisher')

        # Specify data type and topic name. Specify queue size (limit amount of queued messages)
        self.publisher_ = self.create_publisher(ControlMessage, '/odrive_axis0/control_message', 10)

        # Create a timer that will call the 'timer_callback' function every timer_period second.
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def timer_callback(self):
        
        msg = ControlMessage()
        
        msg.control_mode = ControlMode.VELOCITY_CONTROL
        msg.input_mode = InputMode.VEL_RAMP
        msg.input_vel = float(50)

        # Publish the message.
        self.publisher_.publish(msg)


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
