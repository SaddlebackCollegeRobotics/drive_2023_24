# from odrive_can.srv import AxisState
# import rclpy
# from rclpy.node import Node


# class MinimalClientAsync(Node):

#     def __init__(self):
#         super().__init__('minimal_client_async')

#         self.cli = self.create_client(AxisState, 'request_axis_state')

#         while not self.cli.wait_for_service(timeout_sec=1.0):
#             self.get_logger().info('service not available, waiting again...')

#         self.req = AxisState.Request()

#     def send_request(self, state):

#         self.req.axis_requested_state = state

#         self.future = self.cli.call_async(self.req)
#         rclpy.spin_until_future_complete(self, self.future)
#         return self.future.result()


# def request_state():

#     minimal_client = MinimalClientAsync()
#     response = minimal_client.send_request(state=8)
#     minimal_client.destroy_node()

#     return response