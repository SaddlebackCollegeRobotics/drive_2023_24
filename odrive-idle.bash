# Set all motors to idle

echo "Setting all motors to idle"

ros2 service call /odrive_axis0/request_axis_state odrive_can/srv/AxisState "{axis_requested_state: 1}"
ros2 service call /odrive_axis1/request_axis_state odrive_can/srv/AxisState "{axis_requested_state: 1}"
ros2 service call /odrive_axis2/request_axis_state odrive_can/srv/AxisState "{axis_requested_state: 1}"
ros2 service call /odrive_axis3/request_axis_state odrive_can/srv/AxisState "{axis_requested_state: 1}"
