import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist, Vector3

from time import sleep

from itertools import count



class SpiralDriver(Node):

    # Frequency at which this node will publish messages, in Hz
    LOOP_RATE: float = 15

    # Speed to rotate at, rad/s
    # This is for IN-PLACE rotation
    ROTATION_SPEED: float = 0.5

    def __init__(self):
        super().__init__('spiral_driver')

        print("Spiral driver node started.")

        self._publisher = self.create_publisher(Float64MultiArray, '/drive/control_input', 10)

        self._vel_msg = Float64MultiArray()

        # loop_rate = self.create_rate(SpiralDriver.LOOP_RATE, self.get_clock())

        # Temp delay for operator to clear area
        for _ in range(20):
            self._vel_msg.data = [0.0, 0.0]
            self._publisher.publish(self._vel_msg)
            sleep(0.5)

        # Send messages to rotate for a full second
        TURN_IN_PLACE_TIME: int = 7
        from math import pi
        for _ in range(TURN_IN_PLACE_TIME * SpiralDriver.LOOP_RATE):
            self.send_rotate_msg(2 * pi / TURN_IN_PLACE_TIME)
            sleep(1/SpiralDriver.LOOP_RATE)

        # Keep spiraling forever, keep track of current iteration to make spiral larger
        #TODO: Change this sometime
        CURVE_RATE: float = 999/1000
        current_curve_amt: float = 99/100
        while True:
            self.send_spiral_msg(current_curve_amt)
            # loop_rate.sleep()
            sleep(1/SpiralDriver.LOOP_RATE)
            current_curve_amt *= CURVE_RATE

    def send_rotate_msg(self, speed: float):
        """This method sends a Twist message to rotate the rover in-place,
        dependent on the specified speed.

        Args:
            speed (float): Angular speed to rotate at, in rad/s.
        """
        print(f"Sending rotate message with speed {speed}")
        self._vel_msg.data = [-1.0, 1.0]

        self._publisher.publish(self._vel_msg)
    
    def send_spiral_msg(self, curve_amt: float):
        """This method sends a Twist message to drive the rover at a unit vector forward and
        the specified `rotation_speed`. Rotation speed should be decreased over time as to
        create a spiral pattern.

        Args:
            rotation_speed (float): How fast the rover should rotate in rad/s.
        """
        # print(f"Sending spiral message with linear speed {linear_speed} and rotation speed {rotation_speed}")
        self._vel_msg.data = [1.0 - 1.0 * curve_amt, 1.0 ]

        self._publisher.publish(self._vel_msg)




def main(args=None):
    rclpy.init(args=args)

    spiral_driver = SpiralDriver()

    rclpy.spin(spiral_driver)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
