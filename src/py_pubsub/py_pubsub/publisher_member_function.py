import rclpy
from rclpy.node import Node

from odrive_can.msg import ControlMessage
from odrive.enums import InputMode
from odrive.enums import ControlMode
from . import gamepad_input
import os
from numpy import clip


class MotorController():
    
    def __init__(self, node: Node, topic_name: str, max_speed: float):
        self._axis_publisher = node.create_publisher(ControlMessage, topic_name, 10)

        self._control_msg = ControlMessage()
        self._control_msg.control_mode = ControlMode.VELOCITY_CONTROL
        self._control_msg.input_mode = InputMode.PASSTHROUGH
        self._control_msg.input_vel = 0.0

        self._max_speed = max_speed

    def set_velocity(self, vel: float):
        self._control_msg.input_vel = float(clip(vel, -self._max_speed, self._max_speed))
        self._axis_publisher.publish(self._control_msg)

    def set_normalized_velocity(self, normalized_analog_input: float):
        self.set_velocity(normalized_analog_input * self._max_speed)

    def set_max_speed(self, max_speed: float):
        self._max_speed = abs(max_speed)

    def change_max_speed(self, delta: float):
        new_speed = self._max_speed + delta
        self.set_max_speed(0 if new_speed < 0 else new_speed)

        

class MinimalPublisher(Node):

    def __init__(self):

        # Give the node a name.
        super().__init__('minimal_publisher')

        # Create a timer that will call the 'timer_callback' function every timer_period second.
        timer_period = 1/10  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Set up gamepad input --------------------------------------------------------

        self.gamepad_deadzone = 0.1

        # TODO use ros getshare path function instead of doing this
        config_path = os.path.join(os.path.dirname(os.path.realpath(__file__)),
                    '../../../../share/py_pubsub/gamepads.config')

        gamepad_input.setConfigFile(config_path)

        hatEvents = [self.hatNorth, self.hatSouth, None, None, None]
        gamepad_input.run_event_loop(hatEvents=hatEvents)

        # Set up motor controller publishers ------------------------------------------

        INITIAL_MAX_SPEED = 30

        self.motor_controllers = [
            MotorController(self, '/odrive_axis0/control_message', INITIAL_MAX_SPEED),
            MotorController(self, '/odrive_axis1/control_message', INITIAL_MAX_SPEED),
            MotorController(self, '/odrive_axis2/control_message', INITIAL_MAX_SPEED),
            MotorController(self, '/odrive_axis3/control_message', INITIAL_MAX_SPEED)
        ]

        
    def timer_callback(self):
        
        gamepad = gamepad_input.getGamepad(0)

        if gamepad != None:
            (_, ls_y) = gamepad_input.getLeftStick(gamepad, self.gamepad_deadzone)
            (_, rs_y) = gamepad_input.getRightStick(gamepad, self.gamepad_deadzone)

            ls_y = -ls_y

            self.motor_controllers[0].set_normalized_velocity(ls_y)
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
