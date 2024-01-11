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

# param = 'config.brake_resistor0.enable_dc_bus_voltage_feedback'
# value = True

# motor_controller_manager.write_param_all(param, value)

# param = 'config.brake_resistor0.dc_bus_voltage_feedback_ramp_start'
# value = 40

# motor_controller_manager.write_param_all(param, value)

# param = 'config.brake_resistor0.dc_bus_voltage_feedback_ramp_end'
# value = 46

# motor_controller_manager.write_param_all(param, value)

param = 'axis0.controller.config.vel_ramp_rate'
value = 75

motor_controller_manager.write_param_all(param, value)

motor_controller_manager.save_configuration_all()

