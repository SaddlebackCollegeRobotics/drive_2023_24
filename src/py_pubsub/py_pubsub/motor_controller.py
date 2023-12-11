from rclpy.node import Node

from odrive_can.msg import ControlMessage

from odrive.enums import InputMode
from odrive.enums import ControlMode
from numpy import clip
from threading import Thread
import subprocess


class MotorController():
    
    def __init__(self, node: Node, topic_name: str, service_name: str, max_speed: float):
        self._axis_publisher = node.create_publisher(ControlMessage, topic_name, 10)

        self._control_msg = ControlMessage()
        self._control_msg.control_mode = ControlMode.VELOCITY_CONTROL
        self._control_msg.input_mode = InputMode.PASSTHROUGH
        self._control_msg.input_vel = 0.0

        self._max_speed = max_speed

        thread = Thread(target=self.enter_closed_loop, args=(service_name,))
        thread.start()

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

    def enter_closed_loop(self, service_name: str):
        subprocess.run(["ros2", "service", "call", service_name, "odrive_can/srv/AxisState", "{axis_requested_state: 8}"])
