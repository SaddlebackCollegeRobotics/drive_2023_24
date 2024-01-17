import json
import can
import struct
from time import sleep, time
from odrive.enums import AxisState, ProcedureResult, AxisError, ODriveError
from enum import IntEnum
from numpy import clip
from pathlib import Path
from typing import Callable, Any

# TODO: Improved logging system.

class MotorControllerManager:

    _motor_controllers: dict[str, "MotorController"]
    
    def __init__(self, can_interface: "ODriveCanInterface") -> None:

        # Key: node id, Value: MotorController
        self._motor_controllers = {}
        self._can_interface = can_interface

    def add_motor_controller(self, name: str, node_id: int, max_speed: float):
        if name not in self._motor_controllers:
            self._motor_controllers[name] = MotorController(self._can_interface, node_id, max_speed)
        else:
            raise Exception(f"Motor controller with name {name} already exists")
        
    def get_motor_controller(self, name: str):
        return self._motor_controllers[name]

    def set_all(self, func: Callable, *args):
        for motor_controller in self._motor_controllers.values():
            func(motor_controller, *args)
    
    # TODO: Refactor
    def set_axis_state_all(self, axis_state: AxisState):
        for motor_controller in self._motor_controllers.values():
            motor_controller.set_axis_state(axis_state)

    # TODO: Refactor
    def set_input_vel_all(self, vel: float):
        for motor_controller in self._motor_controllers.values():
            motor_controller.set_velocity(vel)

    # TODO: Refactor
    def set_normalized_input_vel_all(self, normalized_analog_input: float):
        for motor_controller in self._motor_controllers.values():
            motor_controller.set_normalized_velocity(normalized_analog_input)

    # TODO: Refactor
    def feed_watchdog_all(self):
        for motor_controller in self._motor_controllers.values():
            motor_controller.feed_watchdog()
    
    def count(self):
        return len(self._motor_controllers)

    # Dunder methods as wrappers
    
    def __getitem__(self, key: str) -> "MotorController":
        """Accesses motor controller by specified name (key).

        Args:
            key (str): Name of the motor controller as assigned by `get_motor_controller(...)`.

        Returns:
            MotorController: Motor controller instance to be used.
        """
        return self.get_motor_controller(key)
    
    def __len__(self) -> int:
        """Returns number of motor controllers being handled.
        """
        return self.count()


class ODriveCanInterface():


    OPCODE_READ = 0x00
    OPCODE_WRITE = 0x01

    class COMMAND(IntEnum):
        GET_VERSION = 0x00
        HEARTBEAT = 0x01
        RX_SDO = 0x04
        TX_SDO = 0x05
        SET_AXIS_STATE = 0x07
        SET_INPUT_VEL = 0x0d
        # TODO: Add remaining commands

    def __init__(self, interface: str = 'can0',
                endpoint_lookup_file: str = 'flat_endpoints.json') -> None:
        
        path = Path(__file__).parent / endpoint_lookup_file
        
        with path.open('r') as f:
            self.endpoint_data = json.load(f)
            self.endpoints = self.endpoint_data['endpoints']

        # Odrive CAN node ID
        # self.bus = can.interface.Bus(interface='virtual')# interface, bustype='socketcan',)
        self.bus = can.interface.Bus(interface, bustype='socketcan',)

        # See https://docs.python.org/3/library/struct.html#format-characters
        self.format_lookup = {
            'bool': '?',
            'uint8': 'B', 'int8': 'b',
            'uint16': 'H', 'int16': 'h',
            'uint32': 'I', 'int32': 'i',
            'uint64': 'Q', 'int64': 'q',
            'float': 'f'
        }
    
    def __del__(self) -> None:
        self.bus.shutdown()

    def flush_rx_buffer(self) -> None:
        # Flush CAN RX buffer so there are no more old pending messages
        while not (self.bus.recv(timeout=0) is None): pass

    def _assert_version(self, node_id: int) -> None:
        # Flush CAN RX buffer so there are no more old pending messages
        self.flush_rx_buffer()

        self.send_can_message(node_id, ODriveCanInterface.COMMAND.GET_VERSION)

        msg = self._await_can_reply(node_id, ODriveCanInterface.COMMAND.GET_VERSION)

        _, hw_product_line, hw_version, hw_variant, fw_major, fw_minor, fw_revision, fw_unreleased = self._unpack_can_reply('<BBBBBBBB', msg)

        # If one of these asserts fail, you're probably not using the right flat_endpoints.json file
        assert self.endpoint_data['fw_version'] == f"{fw_major}.{fw_minor}.{fw_revision}"
        assert self.endpoint_data['hw_version'] == f"{hw_product_line}.{hw_version}.{hw_variant}"


    def _unpack_can_reply(self, return_types: str, msg: bytearray) -> tuple:
        return struct.unpack(return_types, msg.data)


    def _await_can_reply(self, node_id: int, command_id: "ODriveCanInterface.COMMAND", timeout: float = 5.0) -> can.Message | None:

        initial_time = time()
        for msg in self.bus: # FIXME: Improper check for timeout
            if msg.arbitration_id == (node_id << 5 | command_id):
                return msg 
            if time() - initial_time > timeout:
                print(f"Timeout waiting for CAN reply for command {ODriveCanInterface.COMMAND(command_id).name}")
                return None

    def send_can_message(self, node_id: int, command_id: "ODriveCanInterface.COMMAND", input_types: str = "", *input_data: Any) -> None:
        self.bus.send(can.Message(
                arbitration_id = (node_id << 5 | command_id),
                data = struct.pack(input_types, *input_data) if input_types else b'',
                is_extended_id = False
        ))

    def _get_endpoint_info(self, path: str) -> tuple[int, str]:
        endpoint_id = self.endpoints[path]['id']
        endpoint_type = self.endpoints[path]['type']

        return endpoint_id, endpoint_type
    
    def write_param(self, node_id: int, path: str, value: Any) -> None:
        endpoint_id, endpoint_type = self._get_endpoint_info(path)

        self.send_can_message(node_id, ODriveCanInterface.COMMAND.RX_SDO, '<BHB' + self.format_lookup[endpoint_type], self.OPCODE_WRITE, endpoint_id, 0, value)

    def read_param(self, node_id: int, path: str) -> Any:
        endpoint_id, endpoint_type = self._get_endpoint_info(path)

        self.flush_rx_buffer()

        self.send_can_message(node_id, ODriveCanInterface.COMMAND.RX_SDO, '<BHB', self.OPCODE_READ, endpoint_id, 0)

        msg = self._await_can_reply(node_id, ODriveCanInterface.COMMAND.TX_SDO)

        _, _, _, return_value = self._unpack_can_reply('<BHB' + self.format_lookup[endpoint_type], msg)

        return return_value
    
    def send_function_call(self, node_id, path: str, inputs: tuple = None, outputs: tuple = None):
        endpoint_id, _ = self._get_endpoint_info(path)
        self.send_can_message(node_id, ODriveCanInterface.COMMAND.RX_SDO, '<BHB', self.OPCODE_WRITE, endpoint_id, 0)

    def get_heartbeat(self, node_id: int) -> tuple[int, int, int, int]:
        """Get information from ODrive can heartbeat.

        Data returned is (error: uint32, state: uint8, result: uint8, traj_done: uint8)

        Args:
            node_id (int): Id of the can node.

        Returns:
            tuple[int, int, int, int]: Resulting data from heartbeat.
        """
        msg = self._can_interface._await_can_reply(node_id, ODriveCanInterface.COMMAND.HEARTBEAT) # FIXME: May pick up previous
        msg.data = msg.data[:7] # Remove unused bytes
        error, state, result, traj_done = self._can_interface._unpack_can_reply('<IBBB', msg)

        return error, state, result, traj_done


class MotorController():

    _can_interface: ODriveCanInterface
    _node_id: int
    _max_speed: float
    _input_vel: float
    
    def __init__(self, can_interface: ODriveCanInterface, node_id: int, max_speed: float) -> None:

        self._can_interface = can_interface
        self._node_id = node_id
        self._max_speed = max_speed
        self._input_vel = 0.0

    def set_velocity(self, vel: float, torque_feedforward: float = 0.0) -> None:
        self._input_vel = float(clip(vel, -self._max_speed, self._max_speed))
        self._can_interface.send_can_message(self._node_id, ODriveCanInterface.COMMAND.SET_INPUT_VEL, '<ff', vel, torque_feedforward)

    def set_normalized_velocity(self, normalized_analog_input: float) -> None:
        """Sets motor velocity based on a normalized value and max speed.

        Args:
            normalized_analog_input (float): Normalized value [-1.0, 1.0] for setting velocity.
        """
        self.set_velocity(normalized_analog_input * self._max_speed)

    @property
    def max_speed(self) -> float:
        """I'm the 'x' property."""
        return self._max_speed

    @max_speed.setter
    def max_speed(self, value: float) -> None:
        self._max_speed = abs(value)

    def set_axis_state(self, axis_state: AxisState) -> None:
        print(f"Setting axis state to {axis_state.name}")

        self._can_interface.flush_rx_buffer()

        # Clear errors (This also feeds the watchdog as a side effect)
        self._can_interface.clear_errors(self._node_id)

        self._can_interface.send_can_message(self._node_id, ODriveCanInterface.COMMAND.SET_AXIS_STATE, '<I', axis_state)

        while result == ProcedureResult.BUSY:

            self._can_interface.feed_watchdog(self._node_id) # Feed watchdog while waiting for axis to be set

            error, state, result, _ = self._can_interface.get_heartbeat(self._node_id)

            new_axis_state = AxisState(state) # FIXME: Will this ctor work if operation was unsuccessful? 

            print(f"Axis state: {new_axis_state.name}")

            if result == ProcedureResult.SUCCESS:
                print(f"Axis state set successfully {new_axis_state.name}")
            else:
                # Get disarm reason
                print(f"Axis state set failed: {AxisError(error).name}")
                print(f"Current axis state: {new_axis_state.name}")
    
    def save_configuration(self) -> None:
        self._can_interface.send_function_call(self._node_id, 'save_configuration')

    def reboot(self) -> None:
        self._can_interface.send_function_call(self._node_id, 'reboot')

    def get_errors(self) -> tuple[ODriveError, ODriveError]:
        errs = (
            ODriveError(self._can_interface.read_param(self._node_id, 'axis0.active_errors')),
            ODriveError(self._can_interface.read_param(self._node_id, 'axis0.disarm_reason'))
        )
        return errs
    
    def clear_errors(self) -> None:
        self._can_interface.send_function_call(self._node_id, 'clear_errors')
    
    def feed_watchdog(self) -> None:
        self._can_interface.send_function_call(self._node_id, 'axis0.watchdog_feed')

