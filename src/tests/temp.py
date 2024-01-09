command_id = ex: GET_VERSION, RX_SDO, TX_SDO, Set_Axis_State, etc.
node_id
input_types = ex: '<BHB'

def send_command(self, command_id, node_id, input_types, *args):
    self.bus.send(can.Message(
            arbitration_id=(node_id << 5 | command_id),
            data=struct.pack(input_types, *args),
            is_extended_id=False
))
    

self.bus.send(can.Message(
            arbitration_id=(self.node_id << 5 | GET_VERSION),
            data=b'',
            is_extended_id=False
))

self.bus.send(can.Message(
            arbitration_id=(self.node_id << 5 | RX_SDO),
            data=struct.pack('<BHB' + self.format_lookup[endpoint_type], self.OPCODE_WRITE, endpoint_id, 0, value),
            is_extended_id=False
))

# Send read command
self.bus.send(can.Message(
            arbitration_id=(self.node_id << 5 | RX_SDO),
            data=struct.pack('<BHB', self.OPCODE_READ, endpoint_id, 0),
            is_extended_id=False
))

# Send function call
self.bus.send(can.Message(
            arbitration_id=(self.node_id << 5 | RX_SDO),
            data=struct.pack('<BHB', self.OPCODE_WRITE, endpoint_id, 0),
            is_extended_id=False
))

