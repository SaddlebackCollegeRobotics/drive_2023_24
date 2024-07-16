import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import sys

from .odrive_motor_controller_manager import ODriveMotorControllerManager
from .odrive_motor_controller import ODriveMotorController

class MinimalPublisher(Node):

    def __init__(self, cmd_args):

        self._odrive_manager = ODriveMotorControllerManager(
            'can0',
            get_package_share_directory('drive') + '/flat_endpoints.json',
            1000000)

        self._max_speed = 0

        self._odrive_manager.add_motor_controller('front_left', 0, self._max_speed)
        self._odrive_manager.add_motor_controller('back_left', 1, self._max_speed)
        self._odrive_manager.add_motor_controller('back_right', 2, self._max_speed)
        self._odrive_manager.add_motor_controller('front_right', 3, self._max_speed)

        if len(cmd_args) == 2:
            # Obtain and print ODrive errors
            print("Errors: ", self._odrive_manager.for_each(ODriveMotorController.get_errors))

        if len(cmd_args) != 5:
            print("Usage: configure_odrive <all/node_name> <path> <datatype> <value>")
            sys.exit(1)

        # Give the node a name.
        super().__init__('arm_odrive_config_node')

        # Set up motor controllers ---------------------------------------


        choice = cmd_args[1]
        path = cmd_args[2]
        data_type = cmd_args[3]
        value = cmd_args[4]

        match data_type:
            case 'int':
                value = int(value)
            case 'float':
                value = float(value)
            case 'bool':
                value = (value.lower() == 'true')
            case 'str':
                value = str(value)
            case _:
                print(f"Unknown datatype: {cmd_args[4]}")
                sys.exit(1)

        print(f'choice: {choice}, path: {path}, datatype: {data_type}, value: {value}')


        if choice == 'all':

            # Write param values
            self._odrive_manager.for_each(ODriveMotorController.write_param,
                                            path, value)

            # # Read back param values
            # self._odrive_manager.for_each(ODriveMotorController.read_param, path)
            # name_list = self._odrive_manager._motor_controllers.keys()

            # for name, param in zip(name_list, param_list):
            #     print(f"{name} | {path} = {param}")


            # print(f'choice: {choice}, path: {path}, datatype: {data_type}, value: {value}')
        elif choice in self._odrive_manager._motor_controllers:

            # Set param value
            self._odrive_manager.get_motor_controller(choice).write_param(path, value)

            # # Read back param value
            # param = self._odrive_manager.get_motor_controller(choice).read_param(path)
            # print(f"{choice} | {path} = {param}")

        else:
            print(f"Unknown choice: {cmd_args[1]}")
            sys.exit(1)

        exit(0)

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher(sys.argv)

    try:
        rclpy.spin(minimal_publisher)
    except Exception as e:
        print('Error could not set parameter')

    # Destroy the node explicitly
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
