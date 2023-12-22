# For xr1_hw_interface_node

## Parameters
- port (string) - a string to the name of the UART/Serial port to connect to
  - port: /dev/ttyACM0
- register_passive_wheels (bool) - if passive wheels are even registered to ros_control
- strict_motors_check (bool) - true: will make sure all configured IDs are connected, will kill node if not all are found; false: will set unavailable IDs as mock motors (will not send canbus commands)
- middle_wheel_mimics_front (bool) - if middle wheels (passive) copies state of front wheels, unused if register_passive_wheels==false
- id_front_left (int) - vesc/canbus id of the motor, MUST fit 8bits (0~255)
- id_rear_left (int) - vesc/canbus id of the motor, MUST fit 8bits (0~255)
- id_front_right (int) - vesc/canbus id of the motor, MUST fit 8bits (0~255)
- id_rear_right (int) - vesc/canbus id of the motor, MUST fit 8bits (0~255)
  