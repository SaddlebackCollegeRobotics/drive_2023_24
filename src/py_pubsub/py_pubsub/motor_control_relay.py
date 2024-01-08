import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

# from .motor_controller import MotorControllerManager
from can_manager import OdriveCanManager
from odrive.enums import AxisState

from time import sleep


class MinimalPublisher(Node):

    def __init__(self):

        # Give the node a name.
        super().__init__('motor_control_relay')

        self.control_input_subscriber = self.create_subscription(Float64MultiArray, '/control/drive_control_input', self.control_input_callback, 10)

        # Set up motor controllers ---------------------------------------

        self._max_velocity = 30

        self._manager = OdriveCanManager([0, 1, 2, 3])
        
        
        self._manager.set_axis_state_all(AxisState.CLOSED_LOOP_CONTROL)
        self._manager.set_parameter([0, 2], 'identify', True)

    def control_input_callback(self, msg: Float64MultiArray):

        norm_vel_left, norm_vel_right = msg.data[0], msg.data[1]
        
        if abs(norm_vel_left) > 1 or abs(norm_vel_right) > 1:
            raise ValueError

        self._manager.set_velocity([0, 1], norm_vel_left * self._max_velocity)
        self._manager.set_velocity([2, 3], norm_vel_right * self._max_velocity)

        

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
