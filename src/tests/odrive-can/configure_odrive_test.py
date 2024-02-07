from motor_controller import ODrive_CAN_Interface, MotorControllerManager
from time import sleep
from odrive.enums import AxisState

can_interface = ODrive_CAN_Interface('can0')
motor_controller_manager = MotorControllerManager(can_interface)


INITIAL_MAX_SPEED = 30
motor_controller_manager.add_motor_controller('azimuth', 4, INITIAL_MAX_SPEED)
motor_controller_manager.add_motor_controller('shoulder', 5, INITIAL_MAX_SPEED)
motor_controller_manager.add_motor_controller('elbow', 6, INITIAL_MAX_SPEED)
motor_controller_manager.add_motor_controller('pitch', 7, INITIAL_MAX_SPEED)
motor_controller_manager.add_motor_controller('yaw', 8, INITIAL_MAX_SPEED)
motor_controller_manager.add_motor_controller('roll', 9, INITIAL_MAX_SPEED)

motor_controller_manager.write_param_all('identify', False)

