from numpy import clip
from odrive.enums import AxisState, ProcedureResult, AxisError, ODriveError
from .odrive_can_interface import ODriveCanInterface
from typing import Any


class ODriveMotorController():

    _can_interface: ODriveCanInterface
    _node_id: int
    _max_speed: float
    _input_vel: float
    _name: str

    def __init__(self, can_interface: ODriveCanInterface, name: str, node_id: int, max_speed: float) -> None:
        """Class to manage individual ODrive based motor contollers.

        Args:
            can_interface (ODriveCanInterface): A can interface instance for lower-level calls.
            node_id (int): ID of the can node.
            max_speed (float): Maximum speed to limit the motor controller to.
                May also be accessed with the `max_speed` property.
        """
        self._can_interface = can_interface
        self._name = name
        self._node_id = node_id
        self._max_speed = max_speed
        self._input_vel = 0.0

        # self._can_interface._assert_version(self._node_id)

    def set_velocity(self, vel: float, torque_feedforward: float = 0.0) -> None:
        """Sets ODrive input velocity, limited within [-max_speed, max_speed].

        Args:
            vel (float): ODrive input_velocity.
            torque_feedforward (float, optional): `torque_ff` value. Defaults to 0.0.
        """
        self._input_vel = float(clip(vel, -self._max_speed, self._max_speed))
        self._can_interface.send_can_message(self._node_id, ODriveCanInterface.COMMAND.SET_INPUT_VEL, '<ff', vel, torque_feedforward)

    def set_normalized_velocity(self, normalized_analog_input: float) -> None:
        """Sets motor velocity based on a normalized value and max speed.

        Args:
            normalized_analog_input (float): Normalized value [-1.0, 1.0] for setting velocity.
            Values outside this range will be clipped.
        """

        self.set_velocity(float(clip(normalized_analog_input, -1.0, 1.0)) * self._max_speed)

    @property
    def max_speed(self) -> float:
        """Max speed of the motor controller (in BOTH +/- directions)."""
        return self._max_speed

    @max_speed.setter
    def max_speed(self, value: float) -> None:
        self._max_speed = abs(value)

    def set_axis_state(self, axis_state: AxisState) -> None:
        """Sets the axis state of the motor.

        After access state setter is called, this method will await the next `heartbeat` message
        from the motor. Based on this response, the operation will be deemed
        successful/unsuccessful. Currently, this result is only logged to stdout.

        Args:
            axis_state (AxisState): Axis state to set the ODrive to.
        """
        # logger.debug(f"Setting axis state to {axis_state.name}")

        self._can_interface.flush_rx_buffer()

        # Clear errors (This also feeds the watchdog as a side effect)
        self.clear_errors()

        self._can_interface.send_can_message(self._node_id, ODriveCanInterface.COMMAND.SET_AXIS_STATE, '<I', axis_state)

        result = ProcedureResult.BUSY
        while result == ProcedureResult.BUSY:

            self.feed_watchdog() # Feed watchdog while waiting for axis to be set

            error, state, result, _ = self._can_interface.get_heartbeat(self._node_id)

            new_axis_state = AxisState(state)
            # logger.debug(f"Axis state: {new_axis_state.name}")

            if result == ProcedureResult.SUCCESS:
                # logger.debug(f"Axis state set successfully {new_axis_state.name}")
                return

        # Get disarm reason
        # logger.error(f"Axis state set failed: {AxisError(error).name}")
        # logger.error(f"Current axis state: {new_axis_state.name}")

    def get_errors(self) -> tuple[ODriveError, ODriveError]:
        """Get ODrive errors.

        Returns:
            tuple[ODriveError, ODriveError]: `active_errors` and `disarm_reason`.
        """
        errs = (
            ODriveError(self._can_interface.read_param(self._node_id, 'axis0.active_errors')),
            ODriveError(self._can_interface.read_param(self._node_id, 'axis0.disarm_reason'))
        )

        return errs

    def clear_errors(self) -> None:
        """Clear ODrive errors.
        """
        self._can_interface.send_function_call(self._node_id, 'clear_errors')

    def feed_watchdog(self) -> None:
        """Feed watchdog.
        """
        self._can_interface.send_function_call(self._node_id, 'axis0.watchdog_feed')

    def save_configuration(self) -> None:
        """Save ODrive configuration values and reboot.
        """
        self._can_interface.send_function_call(self._node_id, 'save_configuration')

    def reboot(self) -> None:
        """Request that ODrive reboots.
        """
        self._can_interface.send_function_call(self._node_id, 'reboot')

    def identify(self, enable: bool) -> None:
        """Identify ODrive.
        Will cause a light to blink on the ODrive.
        """
        self._can_interface.write_param(self._node_id, 'identify', enable)

    def get_encoder_estimates(self):
        return self._can_interface.get_encoder_estimates(self._node_id)

    def write_param(self, path: str, value: Any):
        self._can_interface.write_param(self._node_id, path, value)

    def read_param(self, path: str):
        return self._can_interface.read_param(self._node_id, path)

    def get_name(self):
        return self._name
