import rclpy
from rclpy.node import Node
from .motor_controller_manager import MotorControllerManager
from ament_index_python import get_package_share_directory

from odrive.enums import AxisState

class Test(Node):

    def __init__(self):
        super().__init__('test')

        manager = MotorControllerManager('can0', get_package_share_directory('drive') + '/flat_endpoints.json')
        manager.add_motor_controller('pitch', 7, 3)
        # manager['pitch'].set_axis_state(AxisState.ENCODER_HALL_POLARITY_CALIBRATION)
        # manager['pitch'].set_axis_state(AxisState.ENCODER_HALL_PHASE_CALIBRATION)
        # manager['pitch'].save_configuration()
        manager['pitch'].set_axis_state(AxisState.CLOSED_LOOP_CONTROL)
        manager['pitch'].set_normalized_velocity(1.0)


def main(args=None):

    rclpy.init(args=args)

    test = Test()

    rclpy.spin(test)

    # Destroy the node explicitly
    test.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
