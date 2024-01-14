import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from .motor_controller import MotorControllerManager, ODriveCanInterface
from odrive.enums import AxisState

from time import sleep
from threading import Thread
from signal import signal, SIGINT


class MinimalPublisher(Node):

    def __init__(self):

        # Give the node a name.
        super().__init__('motor_control_relay')

        # Signal handler for Ctrl+C
        # signal(SIGINT, self.signalHandler)

        self.control_input_subscriber = self.create_subscription(Float64MultiArray, '/control/drive_control_input', self.control_input_callback, 10)

        # Set up motor controllers ---------------------------------------

        self._max_speed = 30


        self._manager = MotorControllerManager(can_interface=ODriveCanInterface())
        
        # TODO: Check ordering
        self._manager.add_motor_controller('front_left', 0, self._max_speed)
        self._manager.add_motor_controller('back_left', 1, self._max_speed)
        self._manager.add_motor_controller('back_right', 2, self._max_speed)
        self._manager.add_motor_controller('front_right', 3, self._max_speed)
        
        self._manager.set_axis_state_all(AxisState.CLOSED_LOOP_CONTROL)

        # self.watchdog_thread = Thread(target=self.feed_watchdog_loop, args=(self._manager,))
        # self.watchdog_thread.loop = True

        # self.watchdog_thread.start()

    # def signalHandler(self, signal, frame):
    #     self.watchdog_thread.loop = False
    #     self.watchdog_thread.join()
    #     exit(0)

    # def feed_watchdog_loop(self, manager: MotorControllerManager):
    #     while getattr(thread, "loop", True):
    #         manager.feed_watchdog_all()
    #         sleep(0.5)

    def control_input_callback(self, msg: Float64MultiArray):

        norm_vel_left, norm_vel_right = msg.data[0], msg.data[1]
        
        if abs(norm_vel_left) > 1 or abs(norm_vel_right) > 1:
            raise ValueError

        self._manager['front_left'].set_normalized_velocity(-norm_vel_left)
        self._manager['back_left'].set_normalized_velocity(-norm_vel_left)
        self._manager['front_right'].set_normalized_velocity(norm_vel_right)
        self._manager['back_right'].set_normalized_velocity(norm_vel_right)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
