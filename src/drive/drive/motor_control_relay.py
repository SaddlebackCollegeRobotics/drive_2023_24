import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from .motor_controller_manager import MotorControllerManager, MotorController
from odrive.enums import AxisState
from ament_index_python.packages import get_package_share_directory


class MotorControlRelay(Node):

    def __init__(self):

        super().__init__('motor_control_relay')

        self.control_input_subscriber = self.create_subscription(Float64MultiArray, '/drive/control_input', self.control_input_callback, 10)

        # Set up motor controller intefaces

        self._max_speed = 30

        can_enpoints_file = get_package_share_directory('drive') + '/flat_endpoints.json'

        self._manager = MotorControllerManager('can0', can_enpoints_file, 1000000)
        
        self._manager.add_motor_controller('front_left', 0, self._max_speed)
        self._manager.add_motor_controller('back_left', 1, self._max_speed)
        self._manager.add_motor_controller('back_right', 2, self._max_speed)
        self._manager.add_motor_controller('front_right', 3, self._max_speed)
        
        # Set all motor controllers to closed loop control
        self._manager.for_each(MotorController.set_axis_state, AxisState.CLOSED_LOOP_CONTROL)

    def control_input_callback(self, msg: Float64MultiArray):

        norm_vel_left, norm_vel_right = msg.data[0], msg.data[1]
        
        self._manager['front_left'].set_normalized_velocity(-norm_vel_left)
        self._manager['back_left'].set_normalized_velocity(-norm_vel_left)
        self._manager['front_right'].set_normalized_velocity(norm_vel_right)
        self._manager['back_right'].set_normalized_velocity(norm_vel_right)

        pos_estimate, vel_estimate = self._manager['front_left'].get_encoder_estimates()
        self.get_logger().info(f'Front Left Pos: {pos_estimate}')
        self.get_logger().info(f'Front Left Vel: {vel_estimate}')       


def main(args=None):
    rclpy.init(args=args)

    motor_control_relay = MotorControlRelay()

    rclpy.spin(motor_control_relay)

    # Destroy the node explicitly
    motor_control_relay.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
