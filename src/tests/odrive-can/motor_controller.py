import json
import can
import struct
from time import sleep, time
from odrive.enums import AxisState, ProcedureResult, AxisError, ODriveError 
from numpy import clip

class MotorControllerManager:
    
    def __init__(self, can_interface) -> None:

        # Key: node id, Value: MotorController
        self._motor_controllers: dict[int, MotorController] = {}
        self._can_interface = can_interface

    def add_motor_controller(self, name: str, node_id: int, max_speed: float):
        if name not in self._motor_controllers:
            self._motor_controllers[name] = MotorController(self._can_interface, node_id, max_speed)
        else:
            raise Exception(f"Motor controller with name {name} already exists")
        
    def get_motor_controller(self, name: str):
        return self._motor_controllers[name]
    
    def set_axis_state_all(self, axis_state: AxisState):
        for motor_controller in self._motor_controllers.values():
            motor_controller.set_axis_state(axis_state)

    def set_input_vel_all(self, vel: float):
        for motor_controller in self._motor_controllers.values():
            motor_controller.set_velocity(vel)

    def set_normalized_input_vel_all(self, normalized_analog_input: float):
        for motor_controller in self._motor_controllers.values():
            motor_controller.set_normalized_velocity(normalized_analog_input)
    
    def count(self):
        return len(self._motor_controllers)


class ODrive_CAN_Interface():

    def __init__(self, interface: str = 'can0',
                endpoint_lookup_file: str = 'flat_endpoints.json') -> None:

        with open(endpoint_lookup_file, 'r') as f:
            self.endpoint_data = json.load(f)
            self.endpoints = self.endpoint_data['endpoints']

        # Odrive CAN node ID
        self.bus = can.interface.Bus(interface, bustype='socketcan',)
        
        self.OPCODE_READ = 0x00
        self.OPCODE_WRITE = 0x01

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


    def _assert_can_connection(self):
        
        # Todo - check if CAN connection is alive / able to send and receive messages
        # A get version call may be good.
        pass


    def _assert_version(self, node_id):

        GET_VERSION = 0x00

        # Flush CAN RX buffer so there are no more old pending messages
        self.flush_rx_buffer()

        self._send_can_message(node_id, GET_VERSION)

        msg = self._await_can_reply(node_id, GET_VERSION)

        _, hw_product_line, hw_version, hw_variant, fw_major, fw_minor, fw_revision, fw_unreleased = self._unpack_can_reply('<BBBBBBBB', msg)

        # If one of these asserts fail, you're probably not using the right flat_endpoints.json file
        assert self.endpoint_data['fw_version'] == f"{fw_major}.{fw_minor}.{fw_revision}"
        assert self.endpoint_data['hw_version'] == f"{hw_product_line}.{hw_version}.{hw_variant}"


    def _unpack_can_reply(self, return_types: str, msg):
        return struct.unpack(return_types, msg.data)


    def _await_can_reply(self, node_id, command_id, timeout = 5.0):

        initial_time = time()
        for msg in self.bus:
            if msg.arbitration_id == (node_id<< 5 | command_id):
                return msg 
            if time() - initial_time > timeout:
                raise Exception(f"Timeout waiting for CAN reply for command {command_id}")


    def _send_can_message(self, node_id, command_id, input_types = None, *input_data):

        self.bus.send(can.Message(
                arbitration_id = (node_id << 5 | command_id),
                data = struct.pack(input_types, *input_data) if input_types else b'',
                is_extended_id = False
        ))


    def _get_endpoint_info(self, path: str):
        
        endpoint_id = self.endpoints[path]['id']
        endpoint_type = self.endpoints[path]['type']

        return endpoint_id, endpoint_type
    

    def write_param(self, node_id, path, value):

        RX_SDO = 0x04

        endpoint_id, endpoint_type = self._get_endpoint_info(path)

        self._send_can_message(node_id, RX_SDO, '<BHB' + self.format_lookup[endpoint_type], self.OPCODE_WRITE, endpoint_id, 0, value)


    def read_param(self, node_id, path):
        
        RX_SDO = 0x04
        TX_SDO = 0x05

        endpoint_id, endpoint_type = self._get_endpoint_info(path)

        self.flush_rx_buffer()

        self._send_can_message(node_id, RX_SDO, '<BHB', self.OPCODE_READ, endpoint_id, 0)

        msg = self._await_can_reply(node_id, TX_SDO)

        _, _, _, return_value = self._unpack_can_reply('<BHB' + self.format_lookup[endpoint_type], msg)

        return return_value
    

    def send_function_call(self, node_id, path: str, inputs: tuple = None, outputs: tuple = None):

        RX_SDO = 0x04
        endpoint_id, _ = self._get_endpoint_info(path)
        self._send_can_message(node_id, RX_SDO, '<BHB', self.OPCODE_WRITE, endpoint_id, 0)


    def feed_watchdog(self, node_id) -> None:
        self.send_function_call(node_id, 'axis0.watchdog_feed')


    def clear_errors(self, node_id) -> None:
        self.send_function_call(node_id, 'clear_errors')
    

    def save_configuration(self, node_id) -> None:
        self.send_function_call(node_id, 'save_configuration')


    def reboot(self, node_id) -> None:
        self.send_function_call(node_id, 'reboot')


    def get_errors(self, node_id):
        return self.read_param(node_id, 'axis0.active_errors'), self.read_param(node_id, 'axis0.disarm_reason')


    def set_axis_state(self, node_id, axis_state: AxisState) -> None: 

        SET_AXIS_STATE = 0x07
        HEARTBEAT = 0x01

        print(f"Setting axis state to {axis_state.name}")

        self.flush_rx_buffer()

        # Clear errors (This also feeds the watchdog as a side effect)
        self.clear_errors(node_id)

        self._send_can_message(node_id, SET_AXIS_STATE, '<I', axis_state)

        while True:

            self.feed_watchdog(node_id) # Feed watchdog while waiting for axis to be set

            msg = self._await_can_reply(node_id, HEARTBEAT)
            msg.data = msg.data[:7] # Remove unused bytes
            error, state, result, traj_done = self._unpack_can_reply('<IBBB', msg)

            if result != ProcedureResult.BUSY:
                if result == ProcedureResult.SUCCESS:
                    print(f"Axis state set successfully {AxisState(state).name}")
                else:
                    # Get disarm reason
                    print(f"Axis state set failed: {AxisError(error).name}")
                    print(f"Current axis state: {AxisState(state).name}")

                break


    def set_input_vel(self, node_id, velocity: float, torque_feedforward: float = 0.0) -> None:

        SET_INPUT_VEL = 0x0d
        self._send_can_message(node_id, SET_INPUT_VEL, '<ff', velocity, torque_feedforward)


class MotorController():
    
    def __init__(self, can_interface, node_id: int, max_speed: float):

        self._can_interface = can_interface
        self._node_id: int = node_id
        self._max_speed = max_speed
        self._input_vel = 0.0

    def set_velocity(self, vel: float):
        self._input_vel = float(clip(vel, -self._max_speed, self._max_speed))
        self._can_interface.set_input_vel(self._node_id, self._input_vel)

    def set_normalized_velocity(self, normalized_analog_input: float):
        self.set_velocity(normalized_analog_input * self._max_speed)

    def set_max_speed(self, max_speed: float):
        self._max_speed = abs(max_speed)

    def change_max_speed(self, delta: float):
        new_speed = self._max_speed + delta
        self.set_max_speed(0 if new_speed < 0 else new_speed)

    def set_axis_state(self, axis_state: AxisState):
        self._can_interface.set_axis_state(self._node_id, axis_state)

    def save_configuration(self):
        self._can_interface.save_configuration(self._node_id)