import sys

from odrive_can.srv import AxisState
import rclpy
from rclpy.node import Node

# Notes:
# If import does not work, try to move the srv files to the same folder as this file????

class MinimalClientAsync(Node):

    def __init__(self):

        super().__init__('minimal_client_async')

        self.cli = self.create_client(AxisState, '/odrive_axis0/request_axis_state')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
            
        self.req = AxisState.Request()

    def send_request(self, requested_state):
        self.req.axis_requested_state = requested_state
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main():

    rclpy.init()
    minimal_client = MinimalClientAsync()
    
    response = minimal_client.send_request(8) # Set to closed loop
   
    minimal_client.get_logger().info(
        f"Motor state: {response.axis_state} | Result: {response.procedure_result}")

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()