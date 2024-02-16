import ifcfg
import subprocess
import json
import can
import struct
from enum import IntEnum
from time import time
from typing import Any


class ODriveCanInterface():

    class ACCESS_MODE(IntEnum):
        OPCODE_READ = 0x00
        OPCODE_WRITE = 0x01

    class COMMAND(IntEnum):
        GET_VERSION = 0x00
        HEARTBEAT = 0x01
        RX_SDO = 0x04
        TX_SDO = 0x05
        SET_AXIS_STATE = 0x07
        SET_INPUT_VEL = 0x0d
        GET_ENCODER_ESTIMATES = 0x09

    def __init__(self, interface: str = 'can0',
                endpoint_lookup_file: str = 'flat_endpoints.json',
                bitrate: int = 1000000) -> None:
        
        self.bus = None

        try: 
            with open(endpoint_lookup_file, 'r') as endpoints_file:
                
                self.endpoint_data = json.load(endpoints_file)
                self.endpoints = self.endpoint_data['endpoints']

                self._configure_bus_network(interface, bitrate)

                # Odrive CAN node ID
                self.bus = can.ThreadSafeBus(interface, bustype='socketcan',)


                # See https://docs.python.org/3/library/struct.html#format-characters
                self.format_lookup = {
                    'bool': '?',
                    'uint8': 'B', 'int8': 'b',
                    'uint16': 'H', 'int16': 'h',
                    'uint32': 'I', 'int32': 'i',
                    'uint64': 'Q', 'int64': 'q',
                    'float': 'f'
                }
        except FileNotFoundError:
            print(f"Enpoint lookup file {endpoint_lookup_file} not found")
            exit(0)
        
    def __del__(self) -> None:
        if self.bus != None:
            self.bus.shutdown()
    
    def _configure_bus_network(self, interface_name: str, bitrate: int):
        interface = ifcfg.interfaces().get(interface_name)
    
        if interface is None:
            print(f'Interface {interface_name} not found')
            exit(0)

        if 'UP' in interface.get('flags'):
            print(f'Interface {interface_name} is already up')
        else:
            
            subprocess.run(["sudo", "ip", "link", "set",
                            interface_name, "up", "type", "can",
                            "bitrate", str(bitrate)])
            
            print(f'Started interface {interface_name}')

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
                # logger.error(f"Timeout waiting for CAN reply for command {ODriveCanInterface.COMMAND(command_id).name}")
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

        self.send_can_message(node_id, ODriveCanInterface.COMMAND.RX_SDO, '<BHB' + self.format_lookup[endpoint_type], self.ACCESS_MODE.OPCODE_WRITE, endpoint_id, 0, value)

    def read_param(self, node_id: int, path: str) -> Any:
        endpoint_id, endpoint_type = self._get_endpoint_info(path)

        self.flush_rx_buffer()

        self.send_can_message(node_id, ODriveCanInterface.COMMAND.RX_SDO, '<BHB', self.ACCESS_MODE.OPCODE_READ, endpoint_id, 0)

        msg = self._await_can_reply(node_id, ODriveCanInterface.COMMAND.TX_SDO)

        _, _, _, return_value = self._unpack_can_reply('<BHB' + self.format_lookup[endpoint_type], msg)

        return return_value
    
    def send_function_call(self, node_id, path: str, inputs: tuple = None, outputs: tuple = None):
        endpoint_id, _ = self._get_endpoint_info(path)
        self.send_can_message(node_id, ODriveCanInterface.COMMAND.RX_SDO, '<BHB', self.ACCESS_MODE.OPCODE_WRITE, endpoint_id, 0)

    def get_heartbeat(self, node_id: int) -> tuple[int, int, int, int]:
        """Get information from ODrive can heartbeat.

        Data returned is (error: uint32, state: uint8, result: uint8, traj_done: uint8)

        Args:
            node_id (int): ID of the can node.

        Returns:
            tuple[int, int, int, int]: Resulting data from heartbeat.
        """
        msg = self._await_can_reply(node_id, ODriveCanInterface.COMMAND.HEARTBEAT) # FIXME: May pick up previous
        msg.data = msg.data[:7] # Remove unused bytes
        error, state, result, traj_done = self._unpack_can_reply('<IBBB', msg)

        return error, state, result, traj_done
    
    def get_encoder_estimates(self, node_id: int) -> tuple[float, float]:
        """Get ODrive encoder position and velocity estimates.

        Data returned is (pos_estimate: float32, vel_estimate: float32)

        Args:
            node_id (int): ID of the can node.

        Returns:
            tuple[float: pos_estimate, float: vel_estimate]
        """
        msg = self._await_can_reply(node_id, ODriveCanInterface.COMMAND.GET_ENCODER_ESTIMATES)

        pos_estimate, vel_estimate = self._unpack_can_reply('<ff', msg)

        return pos_estimate, vel_estimate
