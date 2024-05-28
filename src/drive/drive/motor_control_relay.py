import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Empty as EmptyMsg, String
from std_srvs.srv import Empty as EmptySrv

from .odrive_motor_controller import ODriveMotorController
from .odrive_motor_controller_manager import ODriveMotorControllerManager
from odrive.enums import AxisState
from ament_index_python.packages import get_package_share_directory
from time import time


class MotorControlRelay(Node):

    def __init__(self):

        super().__init__('drive_motor_control_relay')

        self._control_input_subscriber = self.create_subscription(Float64MultiArray, '/drive/control_input', self.control_input_callback, 10)
        self._heartbeat_subscriber = self.create_subscription(EmptyMsg, '/system/heartbeat', self.reset_heartbeat, 10)
        self._status_publisher = self.create_publisher(String, '/drive/status', 10)
        self.reset_drive_srv = self.create_service(EmptySrv, '/drive/reset_system', self.reset_odrives)

        # Listen and react to system heartbeat ----------------------
        self.HEARTBEAT_TIMEOUT = 2.0
        self.HEARTBEAT_CHECK_PERIOD = 0.5

        self.last_heartbeat_time = 0
        self.is_heartbeat_active = False
        self.heartbeat_check_ = self.create_timer(self.HEARTBEAT_CHECK_PERIOD, self.check_heartbeat)

        # Feed the motor controller watchdogs -----------------------
        self.WATCHDOG_FEED_PERIOD = 0.5
        self.watchdog_feed_timer = self.create_timer(self.WATCHDOG_FEED_PERIOD, self.feed_watchdogs)
        
        # Drive system status ---------------------------------------
        self.DRIVE_STATUS_PERIOD = 5
        self.drive_status_timer = self.create_timer(self.DRIVE_STATUS_PERIOD, self.publish_drive_status)
        self.drive_status_msg = String()

        # Set up motor controllers ---------------------------------------

        self.MAX_SPEED = 80

        can_endpoints_file = get_package_share_directory('drive') + '/flat_endpoints.json'

        self._manager = ODriveMotorControllerManager('can0', can_endpoints_file, 1000000)
        
        self._manager.add_motor_controller('front_left', 0, self.MAX_SPEED)
        self._manager.add_motor_controller('back_left', 1, self.MAX_SPEED)
        self._manager.add_motor_controller('back_right', 2, self.MAX_SPEED)
        self._manager.add_motor_controller('front_right', 3, self.MAX_SPEED)

        # Set all motor controllers to closed loop control
        self._manager.for_each(ODriveMotorController.set_axis_state, AxisState.CLOSED_LOOP_CONTROL)

    def control_input_callback(self, msg: Float64MultiArray):

        if (self.is_heartbeat_active):

            norm_vel_left, norm_vel_right = msg.data[0], msg.data[1]

            self._manager['front_left'].set_normalized_velocity(-norm_vel_left)
            self._manager['back_left'].set_normalized_velocity(-norm_vel_left)
            self._manager['front_right'].set_normalized_velocity(norm_vel_right)
            self._manager['back_right'].set_normalized_velocity(norm_vel_right)

    def reset_odrives(self, request, response):
        """Resets the odrives to closed loop control in case of
        a fatal error such as over-current.
        """
        self._manager.for_each(ODriveMotorController.clear_errors)
        self._manager.for_each(ODriveMotorController.set_axis_state, AxisState.CLOSED_LOOP_CONTROL)
        
        return response

    def reset_heartbeat(self, msg):
        self.last_heartbeat_time = time()

    def check_heartbeat(self):
        
        self.is_heartbeat_active = time() - self.last_heartbeat_time < self.HEARTBEAT_TIMEOUT
        
        if (self.is_heartbeat_active == False):
            self._manager.for_each(ODriveMotorController.set_normalized_velocity, 0.0)

    def feed_watchdogs(self):
        self._manager.for_each(ODriveMotorController.feed_watchdog)

    def publish_drive_status(self):
        self.drive_status_msg.data = self._manager.get_all_odrive_status()
        self._status_publisher.publish(self.drive_status_msg)


def main(args=None):
    rclpy.init(args=args)

    motor_control_relay = MotorControlRelay()

    rclpy.spin(motor_control_relay)

    # Destroy the node explicitly
    motor_control_relay.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
