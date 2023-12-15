from rclpy.node import Node

from odrive_can.msg import ControlMessage

from odrive.enums import InputMode
from odrive.enums import ControlMode
from numpy import clip
from threading import Thread
import subprocess


class MotorControllerManager():

    def __init__(self, node: Node):

        self._node: Node = node

        # Key: node id, Value: MotorController
        self._motor_controllers: dict[int, MotorController] = {}

    def add_motor_controller(self, node_id: int, max_speed: float):
        if node_id not in self._motor_controllers:
            self._motor_controllers[node_id] = MotorController(self._node, node_id, max_speed)
        else:
            print("Motor controller with node id " + str(node_id) + " already exists.")
    
    # Set velocity ------------------------------------------------------
    
    def set_velocity(self, node_id: int, vel: float):
        self._motor_controllers[node_id].set_velocity(vel)

    def set_velocity_all(self, vel: float):
        for motor_controller in self._motor_controllers.values():
            motor_controller.set_velocity(vel)
    
    def set_normalized_velocity(self, node_id: int, normalized_analog_input: float):
        self._motor_controllers[node_id].set_normalized_velocity(normalized_analog_input)

    # Set max speed -----------------------------------------------------

    def set_max_speed(self, node_id: int, max_speed: float):
        self._motor_controllers[node_id].set_max_speed(max_speed)

    def change_max_speed(self, node_id: int, delta: float):
        self._motor_controllers[node_id].change_max_speed(delta)

    def change_max_speed_all(self, delta: float):
        for motor_controller in self._motor_controllers.values():
            motor_controller.change_max_speed(delta)

    # Enter closed loop -------------------------------------------------

    def enter_closed_loop(self, node_id: int):
        self._motor_controllers[node_id].enter_closed_loop()

    def enter_closed_loop_all(self):
        for motor_controller in self._motor_controllers.values():
            motor_controller.enter_closed_loop()


class MotorController():
    
    def __init__(self, node: Node, node_id: int, max_speed: float):

        self._service_name = "/odrive_axis" + str(node_id) + "/request_axis_state"
        self._topic_name = "/odrive_axis" + str(node_id) + "/control_message"

        self._axis_publisher = node.create_publisher(ControlMessage, self._topic_name, 10)

        self._control_msg = ControlMessage()
        self._control_msg.control_mode = ControlMode.VELOCITY_CONTROL
        self._control_msg.input_mode = InputMode.VEL_RAMP
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

    def enter_closed_loop(self):
        command = ["ros2", "service", "call", self._service_name, "odrive_can/srv/AxisState", "{axis_requested_state: 8}"]
        Thread(target = lambda x: subprocess.run(x), args=(command,)).start()


