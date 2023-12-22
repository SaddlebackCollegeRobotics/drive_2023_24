from rclpy.node import Node

from odrive_can.msg import ControlMessage
from odrive_can.srv import AxisState as AxisStateService
from odrive.enums import AxisState as AxisStateEnum

from odrive.enums import InputMode
from odrive.enums import ControlMode
from numpy import clip

import ifcfg
import subprocess


class MotorControllerManager():

    def __init__(self, node: Node, interface_name: str, bitrate: int):

        self._node: Node = node

        # Key: node id, Value: MotorController
        self._motor_controllers: dict[int, MotorController] = {}

        self.start_can_interface(interface_name, bitrate)

    def add_motor_controller(self, node_id: int, max_speed: float):
        if node_id not in self._motor_controllers:
            self._motor_controllers[node_id] = MotorController(self._node, node_id, max_speed)
        else:
            self.get_logger().warning(f'Motor controller with node id {node_id} already exists.')
    
    def count(self):
        return len(self._motor_controllers)

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
        self._motor_controllers[node_id].set_axis_state(AxisStateEnum.CLOSED_LOOP_CONTROL)

    def enter_closed_loop_all(self):
        for motor_controller in self._motor_controllers.values():
            motor_controller.set_axis_state(AxisStateEnum.CLOSED_LOOP_CONTROL)

    # Calibration -------------------------------------------------------

    def full_calibration_sequence(self, node_id: int):
        self._motor_controllers[node_id].set_axis_state(AxisStateEnum.FULL_CALIBRATION_SEQUENCE)

    def full_calibration_sequence_all(self):
        for motor_controller in self._motor_controllers.values():
            motor_controller.set_axis_state(AxisStateEnum.FULL_CALIBRATION_SEQUENCE)

    # CAN bus configuration ---------------------------------------------
    
    def start_can_interface(self, interface_name: str, bitrate: int):

        interface_dict = ifcfg.interfaces().get(interface_name)
    
        if interface_dict is None:
            self._node.get_logger().error(f'Interface {interface_name} not found!')
            exit(0)

        if 'UP' in interface_dict.get('flags'):
            self._node.get_logger().info(f'Interface {interface_name} is already enabled.')
        else:
            subprocess.run(["sudo", "ip", "link", "set",
                            interface_name, "up", "type", "can",
                            "bitrate", str(bitrate)])
            
            self._node.get_logger().info(f'Started interface {interface_name}.')


class MotorController():
    
    def __init__(self, node: Node, node_id: int, max_speed: float):

        self._node: Node = node
        self._node_id: int = node_id

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
        self.set_velocity(clip(normalized_analog_input, -1, 1) * self._max_speed)

    def set_max_speed(self, max_speed: float):
        self._max_speed = abs(max_speed)

    def change_max_speed(self, delta: float):
        new_speed = self._max_speed + delta
        self.set_max_speed(0 if new_speed < 0 else new_speed)

    def set_axis_state(self, axis_state: AxisStateEnum):

        client = self._node.create_client(AxisStateService, self._service_name)

        while not client.wait_for_service(timeout_sec=1.0):
            self._node.get_logger().warning('Waiting for service ' + self._service_name)

        request = AxisStateService.Request()
        request.axis_requested_state = axis_state

        future = client.call_async(request)
        future.add_done_callback(self._axis_state_callback) #need partial??

    def _axis_state_callback(self, future):
        try:
            # Fix this logic, always true if reposnse returns
            response = future.result()
            self._node.get_logger().info(f'Set axis ({self._node_id}) state: {AxisStateEnum(response.axis_state).name}')
        except Exception as e:
            self._node.get_logger().error(f'Failed to set axis ({self._node_id}) state: Current state {AxisStateEnum(response.axis_state).name}')


