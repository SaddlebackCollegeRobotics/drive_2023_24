import rclpy
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory

from .motor_controller import MotorControllerManager

from . import gamepad_input
import os

class MinimalPublisher(Node):

    def __init__(self):

        # Give the node a name.
        super().__init__('minimal_publisher')

        self.package_share_directory = get_package_share_directory('py_pubsub')

        # Set up motor controllers ---------------------------------------

        INITIAL_MAX_SPEED = 30

        self.motor_controller_manager = MotorControllerManager(self)
        for i in range(4):
            self.motor_controller_manager.add_motor_controller(node_id=i, max_speed=INITIAL_MAX_SPEED)

        self.motor_controller_manager.enter_closed_loop_all()

        # --------------------------------------------------------------------------

        # Set up gamepad input
        self.configure_gamepad_input()

        timer_period = 1/10 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        gamepad = gamepad_input.getGamepad(0)

        if gamepad != None:
            (_, ls_y) = gamepad_input.getLeftStick(gamepad, self.GAMEPAD_DEADZONE)
            (_, rs_y) = gamepad_input.getRightStick(gamepad, self.GAMEPAD_DEADZONE)
            (_, r2) = gamepad_input.getTriggers(gamepad, self.GAMEPAD_DEADZONE)

            if r2 > 0:
                rs_y = ls_y

            self.motor_controller_manager.set_normalized_velocity(0, -ls_y)
            self.motor_controller_manager.set_normalized_velocity(1, ls_y)
            self.motor_controller_manager.set_normalized_velocity(2, rs_y)
            self.motor_controller_manager.set_normalized_velocity(3, rs_y)
        else:
            self.motor_controller_manager.set_velocity_all(0)

    def configure_gamepad_input(self):
        
        self.GAMEPAD_DEADZONE = 0.1

        config_path = os.path.join(self.package_share_directory, 'gamepads.config')
        gamepad_input.setConfigFile(config_path)

        MAX_SPEED_DELTA = 5
        hatNorth = lambda: self.motor_controller_manager.change_max_speed_all(MAX_SPEED_DELTA)
        hatSouth = lambda: self.motor_controller_manager.change_max_speed_all(-MAX_SPEED_DELTA)

        hatEvents = [hatNorth, hatSouth, None, None, None]
        gamepad_input.run_event_loop(hatEvents=hatEvents)

    


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
