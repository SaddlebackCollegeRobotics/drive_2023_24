import json
import can
import struct
from time import sleep
from odrive.enums import AxisState, ProcedureResult, AxisError, ODriveError 

class ODriveCANInterface:
    
    def __init__(self, node_id: int = 0, interface: str = 'can0',
                endpoint_lookup_file: str = 'flat_endpoints.json') -> None:

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
        self.bus.shutdown()


    def flush_rx_buffer(self) -> None:
        # Flush CAN RX buffer so there are no more old pending messages
        while not (self.bus.recv(timeout=0) is None): pass


    def _assert_version(self):

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


    def send_write_command(self, path, value_to_write):

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


    def send_read_command(self, path):
        
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


    # For calling functions that don't return a value    
    def send_function_call(self, path):

        RX_SDO = 0x04

        # Convert path to endpoint ID
        endpoint_id = self.endpoints[path]['id']

        self.bus.send(can.Message(
            arbitration_id=(self.node_id << 5 | RX_SDO),
            data=struct.pack('<BHB', self.OPCODE_WRITE, endpoint_id, 0),
            is_extended_id=False
        ))


    def feed_watchdog(self) -> None:
        self.send_function_call('axis0.watchdog_feed')


    def clear_errors(self) -> None:
        self.send_function_call('clear_errors')
    
    
    def save_configuration(self) -> None:
        self.send_function_call('save_configuration')


    def reboot(self) -> None:
        self.send_function_call('reboot')
        

    def get_errors(self):
        return self.send_read_command('axis0.active_errors'), self.send_read_command('axis0.disarm_reason')


    def set_axis_state(self, axis_state: AxisState) -> None: 

        SET_AXIS_STATE = 0x07
        HEARTBEAT = 0x01

        print(f"Setting axis state to {axis_state.name}")

        self.flush_rx_buffer()

        # Clear errors (This also feeds the watchdog as a side effect)
        self.clear_errors()

        # Put axis into full calibration sequence state
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

        # TODO - do we need torque feedforward to be variable?

        SET_INPUT_VEL = 0x0d

        # Velocity is measure in turns/s

        self.bus.send(can.Message(
            arbitration_id=(self.node_id << 5 | SET_INPUT_VEL),
            data=struct.pack('<ff', velocity, 0.0), # 0.0: torque feedforward
            is_extended_id=False
        ))


def main():

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