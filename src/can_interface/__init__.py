"""The ``can_interface`` module provides an easy way to use CAN with ODrives.


"""

import json
from typing import Any
import struct
from odrive.enums import AxisState, ProcedureResult, AxisError, ODriveError
import can

__all__ = [
    'can_bus',
    ''
]

# TODO: Determine preferred usage of interface(s)
class ODriveCanNode:
    pass

class ODriveCANInterface:
    """Handles behaviors of a specific CAN node.
    """
    
    def __init__(self, node_id: int = 0, interface: str = 'can0',
                endpoint_lookup_file: str = 'flat_endpoints.json') -> None:
        """Initializes the CAN interface.
        
        Loads the endpoints lookup file, creates a new can bus instance and
        asserts that the config file is of the correct version.

        Args:
            node_id (int, optional): ID of the motor to manage. Defaults to 0.
                Must match `<odrv>.axis0.config.can.node_id`.
            interface (str, optional): Name of the can port. Defaults to 'can0'.
            endpoint_lookup_file (str, optional): Path to the the endpoint json
                file. Defaults to 'flat_endpoints.json'.
        """
        # TODO: Auto-configure the bus interface status (bring online + bitrate)
        with open(endpoint_lookup_file, 'r') as f:
            self.endpoint_data = json.load(f)
            self.endpoints = self.endpoint_data['endpoints']

        # Odrive CAN node ID
        self.node_id = node_id
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

        # Assert that the endpoint lookup file matches the firmware 
        # and hardware version of the controller.
        self._assert_version()


    def __del__(self) -> None:
        """Shuts down the bus interface.
        """
        self.bus.shutdown()


    def flush_rx_buffer(self) -> None: # TODO: Change to "clear_messages"?
        """Clears pending messages from can bus.
        """
        # Flush CAN RX buffer so there are no more old pending messages
        while not (self.bus.recv(timeout=0) is None): pass


    def _assert_version(self) -> None:
        """Asserts that connected can bus version matches the one in endpoints.
        """

        GET_VERSION = 0x00

        # Flush CAN RX buffer so there are no more old pending messages
        self.flush_rx_buffer()

        # Send read command
        self.bus.send(can.Message(
            arbitration_id=(self.node_id << 5 | GET_VERSION),
            data=b'',
            is_extended_id=False
        ))

        # Await reply
        for msg in self.bus:
            if msg.arbitration_id == (self.node_id << 5 | GET_VERSION):
                break

        _, hw_product_line, hw_version, hw_variant, fw_major, fw_minor, fw_revision, fw_unreleased = struct.unpack('<BBBBBBBB', msg.data)

        # If one of these asserts fail, you're probably not using the right flat_endpoints.json file
        assert self.endpoint_data['fw_version'] == f"{fw_major}.{fw_minor}.{fw_revision}"
        assert self.endpoint_data['hw_version'] == f"{hw_product_line}.{hw_version}.{hw_variant}"


    def send_write_command(self, path, value_to_write) -> None:
        """Sends an command to the can bus node to write to an arbitrary
        parameter.

        Args:
            path (str): Path of the parameter to write.
                Must be specified in endpoint.
            value_to_write: Value to write to specified parameter.
        
        Example:
            Sets the vel_integrator limit to 1.234, then reads from it.
            
            >>> path = 'axis0.controller.config.vel_integrator_limit'
            >>> myInterface.send_write_command(path, 1.234)
            >>> myInterface.send_read_command(path)
            1.234
        """

        RX_SDO = 0x04

        # Convert path to endpoint ID
        endpoint_id = self.endpoints[path]['id']
        endpoint_type = self.endpoints[path]['type']

        # Send write command
        self.bus.send(can.Message(
            arbitration_id=(self.node_id << 5 | RX_SDO),
            data=struct.pack('<BHB' + self.format_lookup[endpoint_type], self.OPCODE_WRITE, endpoint_id, 0, value_to_write),
            is_extended_id=False
        ))


    def send_read_command(self, path: str) -> Any:
        """Sends an command to the can bus node to read from an arbitrary
        parameter.

        Args:
            path (str): Path of the parameter to read from.
                Must be specified in endpoint.
        
        Returns:
            Any: Data retrieved from parameter at `path`.
        
        Example:
            Sets the vel_integrator limit to 1.234, then reads from it.
            
            >>> path = 'axis0.controller.config.vel_integrator_limit'
            >>> myInterface.send_write_command(path, 1.234)
            >>> myInterface.send_read_command(path)
            1.234
        """
        RX_SDO = 0x04
        TX_SDO = 0x05

        # Convert path to endpoint ID
        endpoint_id = self.endpoints[path]['id']
        endpoint_type = self.endpoints[path]['type']

        self.flush_rx_buffer()

        # Send read command
        self.bus.send(can.Message(
            arbitration_id=(self.node_id << 5 | RX_SDO),
            data=struct.pack('<BHB', self.OPCODE_READ, endpoint_id, 0),
            is_extended_id=False
        ))

        # Await reply
        for msg in self.bus:
            if msg.arbitration_id == (self.node_id << 5 | TX_SDO):
                break

        # Unpack and print reply
        _, _, _, return_value = struct.unpack_from('<BHB' + self.format_lookup[endpoint_type], msg.data)

        return return_value


    def send_function_call(self, path):
        """Sends an command to the can bus node to call an arbitrary
        function, which does not return a value

        Args:
            path (str): Path to the function to call.
        """

        RX_SDO = 0x04

        # Convert path to endpoint ID
        endpoint_id = self.endpoints[path]['id']

        self.bus.send(can.Message(
            arbitration_id=(self.node_id << 5 | RX_SDO),
            data=struct.pack('<BHB', self.OPCODE_WRITE, endpoint_id, 0),
            is_extended_id=False
        ))


    # TODO: Simpler watchdog handling for end user. (Create thread automatically?)
    def feed_watchdog(self) -> None:
        """Feeds the watchdog of the node to prevent timeout.
        """
        self.send_function_call('axis0.watchdog_feed')


    def clear_errors(self) -> None:
        """Clears all active errors.
        """
        self.send_function_call('clear_errors')
    
    
    def save_configuration(self) -> None:
        """Saves active configuration of the node.
        """
        self.send_function_call('save_configuration')


    def reboot(self) -> None:
        """Reboots the node.
        """
        self.send_function_call('reboot')
        

    def get_errors(self) -> (int, int) :
        """Gets all active errors as two uint32's, errors set as flags. The
        first value is for general active errors, and second for disarm reason.

        Returns:
            (int, int): Set of active error(s)
        """
        # TODO: Automatically parse errors to be human readable
        return self.send_read_command('axis0.active_errors'), self.send_read_command('axis0.disarm_reason')


    def set_axis_state(self, axis_state: AxisState) -> None:
        """Sets the axis state of the can node.
        This handles any pending messages and basically resets the can node.
        It is ensured that the node is sending heartbeat messages before
        attempting to set state.

        Args:
            axis_state (AxisState): State to seet the node to.
        """
        
        # TODO: Give user return value or raise error to determine if method was successful

        SET_AXIS_STATE = 0x07
        HEARTBEAT = 0x01

        # TODO: Option to configuring log verbosity level
        print(f"Setting axis state to {axis_state.name}")

        self.flush_rx_buffer()

        # Clear errors (This also feeds the watchdog as a side effect)
        self.clear_errors()

        # Set axis state
        self.bus.send(can.Message(
            arbitration_id=(self.node_id << 5 | SET_AXIS_STATE),
            data=struct.pack('<I', axis_state),
            is_extended_id=False
        ))

        # Wait for axis to be set by scanning heartbeat messages
        for msg in self.bus:
            if msg.arbitration_id == (self.node_id << 5 | HEARTBEAT):
                
                self.feed_watchdog() # Feed watchdog while waiting for axis to be set
                error, state, result, traj_done = struct.unpack('<IBBB', bytes(msg.data[:7]))

                if result != ProcedureResult.BUSY:
                    if result == ProcedureResult.SUCCESS:
                        print(f"Axis state set successfully {AxisState(state).name}")
                    else:
                        # Get disarm reason
                        print(f"Axis state set failed: {AxisError(error).name}")
                        print(f"Current axis state: {AxisState(state).name}")

                    break


    def set_input_vel(self, velocity: float) -> None:
        """Sends an input velocity the the can node.
        Behavior will differe depending on the values of vel_limit and 
        input_mode.

        Args:
            velocity (float): Velocity, in rev/s, to send to the can node.
        """

        # TODO - do we need torque feedforward to be variable?

        SET_INPUT_VEL = 0x0d

        # Velocity is measure in turns/s

        self.bus.send(can.Message(
            arbitration_id=(self.node_id << 5 | SET_INPUT_VEL),
            data=struct.pack('<ff', velocity, 0.0), # 0.0: torque feedforward
            is_extended_id=False
        ))


def main():
    from time import sleep
    
    can_interface = ODriveCANInterface(node_id=0, interface='can0', endpoint_lookup_file='flat_endpoints.json')

    path = 'axis0.controller.config.vel_integrator_limit'

    # Read the current velocity integrator limit
    return_value = can_interface.send_read_command(path)
    print(f"Current value: {return_value}")

    # Write a new velocity integrator limit
    value_to_write = 1.234
    can_interface.send_write_command(path, value_to_write)
    print(f"New value written: {value_to_write}")

    # Read the new velocity integrator limit
    return_value = can_interface.send_read_command(path)
    print(f"New value: {return_value}")

    # Reboot the controller to remove the new velocity integrator limit
    can_interface.reboot()

    print('Sleeping for 5 seconds...')
    sleep(5)

    # Run full calibration sequence
    can_interface.set_axis_state(AxisState.FULL_CALIBRATION_SEQUENCE)

    # Save the configuration
    can_interface.save_configuration()
    print("Configuration saved")

    print('Sleeping for 5 seconds...')
    sleep(5)

    # Run closed loop control
    can_interface.set_axis_state(AxisState.CLOSED_LOOP_CONTROL)

    # Set velocity to 5.0 turns/s
    can_interface.set_input_vel(5.0)

    while True:
            can_interface.feed_watchdog()
            sleep(0.01)

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\nExiting...")
