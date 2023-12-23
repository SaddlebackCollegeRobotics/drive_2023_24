import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from .motor_controller import MotorControllerManager


class MinimalPublisher(Node):

    def __init__(self):

        # Give the node a name.
        super().__init__('motor_control_relay')

        self.control_input_subscriber = self.create_subscription(Float64MultiArray, '/control/drive_control_input', self.control_input_callback, 10)

        # Set up motor controllers ---------------------------------------

        INITIAL_MAX_SPEED = 30

        self.motor_controller_manager = MotorControllerManager(node=self, interface_name='can0', bitrate=1000000)
        
        for i in range(4):
            self.motor_controller_manager.add_motor_controller(node_id=i, max_speed=INITIAL_MAX_SPEED)

        self.motor_controller_manager.enter_closed_loop_all()
        # self.motor_controller_manager.full_calibration_sequence(2)
        # self.motor_controller_manager.full_calibration_sequence_all()

    def control_input_callback(self, msg: Float64MultiArray):

        norm_vel_left, norm_vel_right = msg.data[0], msg.data[1]

        self.motor_controller_manager.set_normalized_velocity(0, norm_vel_left)
        self.motor_controller_manager.set_normalized_velocity(1, norm_vel_left)
        self.motor_controller_manager.set_normalized_velocity(2, norm_vel_right)
        self.motor_controller_manager.set_normalized_velocity(3, norm_vel_right)
        

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
