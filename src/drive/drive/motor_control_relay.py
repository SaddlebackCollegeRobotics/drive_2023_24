import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from .motor_controller import MotorControllerManager, ODriveCanInterface, MotorController
from odrive.enums import AxisState

from time import sleep
from threading import Thread
from signal import signal, SIGINT


class MinimalPublisher(Node):

    def __init__(self):

        # Give the node a name.
        super().__init__('motor_control_relay')

        # Signal handler for Ctrl+C
        # signal(SIGINT, self.signalHandler)

        self._control_input_subscriber = self.create_subscription(Float64MultiArray, '/drive/control_input', self.control_input_callback, 10)

        # Set up motor controllers ---------------------------------------

        self._max_speed = 30


        self._manager = MotorControllerManager()
        
        # TODO: Check ordering
        self._manager.add_motor_controller('front_left', 0, self._max_speed)
        self._manager.add_motor_controller('back_left', 1, self._max_speed)
        self._manager.add_motor_controller('back_right', 2, self._max_speed)
        self._manager.add_motor_controller('front_right', 3, self._max_speed)
        
        self._manager.for_each(MotorController.set_axis_state, AxisState.CLOSED_LOOP_CONTROL)

        # Create publisher for motor feedback (position/velocity)
        self._feedback_publisher = self.create_(Float64MultiArray, '/drive/feedback', 10)
        self._feedback_msg = Float64MultiArray() # [pos1, vel1, pos2, vel2, ...]
        self.create_timer(1/10, self.feedback_callback)

    def control_input_callback(self, msg: Float64MultiArray):

        norm_vel_left, norm_vel_right = msg.data[0], msg.data[1]
        
        if abs(norm_vel_left) > 1 or abs(norm_vel_right) > 1:
            raise ValueError

        self._manager['front_left'].set_normalized_velocity(-norm_vel_left)
        self._manager['back_left'].set_normalized_velocity(-norm_vel_left)
        self._manager['front_right'].set_normalized_velocity(norm_vel_right)
        self._manager['back_right'].set_normalized_velocity(norm_vel_right)
    
    def feedback_callback(self):
        # FIXME: The current method of reading parameters is very inefficient
        # Each parameter request will clear current pending msgs, meaning each one must wait
        # for the next cycle.

        feedback = self._manager.for_each(MotorController.get_encoder_est)
        self._feedback_msg.data.clear()
        for motor_feedback_tuple in feedback:
            self._feedback_msg.data += list(motor_feedback_tuple)
        
        print(f'Feedback: {self._feedback_msg.data}')
        
        self._feedback_publisher.publish(self._feedback_msg)



def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
