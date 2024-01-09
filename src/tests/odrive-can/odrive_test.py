from motor_controller import ODrive_CAN_Interface, MotorControllerManager
from time import sleep
from odrive.enums import AxisState

can_interface = ODrive_CAN_Interface('can0')
motor_controller_manager = MotorControllerManager(can_interface)


INITIAL_MAX_SPEED = 30
motor_controller_manager.add_motor_controller('front_left', 0, INITIAL_MAX_SPEED)
motor_controller_manager.add_motor_controller('back_left', 1, INITIAL_MAX_SPEED)
motor_controller_manager.add_motor_controller('back_right', 2, INITIAL_MAX_SPEED)
motor_controller_manager.add_motor_controller('front_right', 3, INITIAL_MAX_SPEED)

motor_controller_manager.set_axis_state_all(AxisState.CLOSED_LOOP_CONTROL)

VELOCITY = 50

while True:
    motor_controller_manager.get_motor_controller('front_left').set_velocity(-VELOCITY)
    motor_controller_manager.get_motor_controller('back_left').set_velocity(-VELOCITY)
    motor_controller_manager.get_motor_controller('back_right').set_velocity(VELOCITY)
    motor_controller_manager.get_motor_controller('front_right').set_velocity(VELOCITY)

    sleep(0.25)
