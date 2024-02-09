from typing import Callable

from .odrive_can_interface import ODriveCanInterface
from .motor_controller import MotorController


class MotorControllerManager:

    _motor_controllers: dict[str, MotorController]
    _can_interface: ODriveCanInterface
    
    def __init__(self, interface, endpoint_lookup_file, bitrate) -> None:
        """Class for managing a group of ODrive motor controllers.
        
        Examples:
            Basic manager example for a two-wheeled robot, with can node ids of [0, 1]:
            Instantiate a manager with an associated can interface.
            Add each of the two motor controllers to be managed.
            Set the axis state of all motor controllers to `CLOSED_LOOP_CONTROL`.
            Set the velocity of the left and right motors to 3 and 5 respectively.

            >>> man = MotorControllerManager()
            >>> man.add_motor_controller("left_wheel", 0, max_speed=10.0)
            >>> man.add_motor_controller("right_wheel", 0, max_speed=5.0)
            >>> man.for_each(MotorController.set_axis_state, AxisState.CLOSED_LOOP_CONTROL)
            >>> man["left_wheel"].set_velocity(3)
            >>> man["right_wheel"].set_velocity(5)
        """
        # Key: node id, Value: MotorController
        self._motor_controllers = {}
        self._can_interface = ODriveCanInterface(interface, endpoint_lookup_file, bitrate)

    def add_motor_controller(self, name: str, node_id: int, max_speed: float) -> None:
        if name not in self._motor_controllers:
            self._motor_controllers[name] = MotorController(self._can_interface, node_id, max_speed)
        else:
            raise Exception(f"Motor controller with name {name} already exists")
        
    def get_motor_controller(self, name: str) -> MotorController:
        return self._motor_controllers[name]

    def for_each(self, func: Callable, *args) -> list | None:
        """Calls a `MotorController` method for each managed controller.
        `*args` must be provided if the specified method requires arguments.

        Args:
            func (Callable): Reference to a `MotorController` method

        Returns:
            list | None: List of return values of each function called. 
        """
        ret = []
        for motor_controller in self._motor_controllers.values():
            if r := func(motor_controller, *args):
                ret.append(r)
        
        return ret if ret else None
    
    def count(self) -> int:
        return len(self._motor_controllers)

    # Dunder Method Wrappers --------------------------------------------------
    
    def __getitem__(self, key: str) -> MotorController:
        """Accesses motor controller by specified name (key).

        Args:
            key (str): Name of the motor controller as assigned by `get_motor_controller(...)`.

        Returns:
            MotorController: Motor controller instance to be used.
        """
        return self.get_motor_controller(key)
    
    def __len__(self) -> int:
        """Returns number of motor controllers being handled.
        """
        return self.count()
    