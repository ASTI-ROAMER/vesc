# For xr1_hw_interface_node

## Parameters
### for VESC motors/wheels
- debug_no_device (bool, false) - will enter debug mode if TRUE, in this mode, NO UART DEVICES ARE NEEDED, for debugging purposes.
- port (string) - a string to the name of the UART/Serial port to connect to VESC controller
  - ex.: port: /dev/ttyACM0
- register_passive_wheels (bool, true) - if passive wheels are even registered to ros_control
  - ex.: register_passive_wheels: true
- strict_motors_check (bool, false) - true: will make sure all configured IDs are connected, will kill node if not all are found; 
  - false: will set unavailable IDs as mock motors (will not send canbus commands)
  - ex.: strict_motors_check: false
- middle_wheel_mimics_front (bool, false) - if middle wheels (passive) copies state of front wheels, unused if register_passive_wheels==false
  - ex.: middle_wheel_mimics_front: false
- id_front_left (int) - vesc/canbus id of the motor, MUST fit 8bits (0~255)
  - ex.:id_front_left: 115
- id_rear_left (int) - vesc/canbus id of the motor, MUST fit 8bits (0~255)
  - ex.:id_rear_left: 23
- id_front_right (int) - vesc/canbus id of the motor, MUST fit 8bits (0~255)
  - ex.:id_front_right: 127
- id_rear_right (int) - vesc/canbus id of the motor, MUST fit 8bits (0~255)
  - ex.:id_rear_right: 67
  
### for leg joint encoders
- leg_port (string) - a string to the name of the UART/Serial port to connect to the arduino with leg encoders
  - ex.: leg_port: /dev/ttyACM0
- use_only_valid_leg_encoder_values (bool, true) - will request data with individual status values for each encoder
  - If true, it will only update the position of the leg joint if it is updated/valid (encoder values will still be copied, but joint position will not be calculated).
  - If false, it will not request the status of individual encoders.
  - ex.: use_only_valid_leg_encoder_values: false

#### for rocker joints (1 encoder to 2 joints, complementary)
- rocker_left_joint_name (string, "rocker_left_joint"): See below.
- rocker_right_joint_name (string, "rocker_right_joint"): Name of the left/ right rocker joint.
- rocker_zero_enc_val (int, 0): Specifies the encoder value for zero degrees. Left and right joint position are complementary, coming only from 1 encoder value. This affects LEFT ROCKER directly. Should be below 12 bits (max 4095).
- rocker_reverse_dir (bool, false): Specifies whether to reverse the joint position (enc_val*-1) calculated from the encoder values. Left and right joint position are complementary, coming only from 1 encoder value. This affects LEFT ROCKER directly.

#### for bogie joints (1 encoder to 1 joint, normal)
- bogie_left_joint_name (string, "bogie_left_joint"): Specify a name for the left bogie joint.
  - ex.: bogie_left_joint_name: "bogie_left_joint"
- bogie_left_zero_enc_val (int, 0): Specifies the encoder value for zero degrees for bogie_left_joint. Should be below 12 bits (max 4095)
  - ex.: bogie_left_zero_enc_val: 23
- bogie_left_reverse_dir (bool, false): Specifies whether to reverse the position (negate) for the joint.
  - ex.: bogie_left_reverse_dir: false

- bogie_right_joint_name (string, "bogie_right_joint"): Specify a name for the right bogie joint.
  - ex.: bogie_right_joint_name: "bogie_right_joint"
- bogie_right_zero_enc_val (int, 0): Specifies the encoder value for zero degrees for bogie_right_joint. Should be below 12 bits (max 4095)
  - ex.: bogie_right_zero_enc_val: 23
- bogie_right_reverse_dir (bool, false): Specifies whether to reverse the position (negate) for the joint.
  - ex.: bogie_right_reverse_dir: false


      