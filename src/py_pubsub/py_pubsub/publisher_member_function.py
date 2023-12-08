import rclpy
from rclpy.node import Node
from .motor_controller import MotorController

import ifcfg

from . import gamepad_input
import os
import subprocess


class MinimalPublisher(Node):

    def __init__(self):

        # Give the node a name.
        super().__init__('minimal_publisher')

        # Check for CAN bus connection
        self.configure_can_bus()
        
        # Set up motor controller publishers ---------------------------------------

        INITIAL_MAX_SPEED = 30

        self.motor_controllers = [
            MotorController(self, '/odrive_axis0/control_message', '/odrive_axis0/request_axis_state', INITIAL_MAX_SPEED),
            MotorController(self, '/odrive_axis1/control_message', '/odrive_axis1/request_axis_state', INITIAL_MAX_SPEED),
            MotorController(self, '/odrive_axis2/control_message', '/odrive_axis2/request_axis_state', INITIAL_MAX_SPEED),
            MotorController(self, '/odrive_axis3/control_message', '/odrive_axis3/request_axis_state', INITIAL_MAX_SPEED)
        ]

        # --------------------------------------------------------------------------

        # Set up gamepad input
        self.configure_gamepad_input()

        timer_period = 1/10  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        
    def timer_callback(self):
        
        gamepad = gamepad_input.getGamepad(0)

        if gamepad != None:
            (_, ls_y) = gamepad_input.getLeftStick(gamepad, self.gamepad_deadzone)
            (_, rs_y) = gamepad_input.getRightStick(gamepad, self.gamepad_deadzone)
            (_, r2) = gamepad_input.getTriggers(gamepad, self.gamepad_deadzone)

            if r2 > 0:
                rs_y = ls_y

            self.motor_controllers[0].set_normalized_velocity(-ls_y)
            self.motor_controllers[1].set_normalized_velocity(ls_y)
            self.motor_controllers[2].set_normalized_velocity(rs_y)
            self.motor_controllers[3].set_normalized_velocity(rs_y)
        else:
            for motor_controller in self.motor_controllers:
                motor_controller.set_velocity(0)

        print("Max Speed: " + str(self.motor_controllers[0]._max_speed))

    def hatNorth(self):
        for motor_controller in self.motor_controllers:
            motor_controller.change_max_speed(5)
    
    def hatSouth(self):
        for motor_controller in self.motor_controllers:
            motor_controller.change_max_speed(-5)

    def configure_gamepad_input(self):
        self.gamepad_deadzone = 0.1

        # TODO use ros getshare path function instead of doing this
        config_path = os.path.join(os.path.dirname(os.path.realpath(__file__)),
                    '../../../../share/py_pubsub/gamepads.config')

        gamepad_input.setConfigFile(config_path)

        hatEvents = [self.hatNorth, self.hatSouth, None, None, None]
        gamepad_input.run_event_loop(hatEvents=hatEvents)

    def configure_can_bus(self):

        found = False
        for name, interface in ifcfg.interfaces().items():
            if name == 'can0':
                found = True
                self.get_logger().info("CAN bus found")
                break  

        if not found:  
            self.get_logger().info("CAN bus not found")
            exit(0)

        # may need sudo?????
        # use subprocess????
        # os.system("sudo ip link set can0 up type can bitrate 1000000")
        subprocess.run(["sudo", "ip", "link", "set", "can0", "up", "type", "can", "bitrate", "1000000"])


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
