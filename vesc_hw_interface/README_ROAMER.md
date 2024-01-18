# For xr1_hw_interface_node

## Parameters
### for VESC motors/wheels
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
- leg{$index}_joint_name (string, ["rocker_left_joint", "bogie_left_joint", "rocker_right_joint", "bogie_right_joint"])
  - Specify a name for each {$index} (0-3) of the joint associated with each encoder value
  - ex.: leg0_joint_name: "bogie_left_joint"
  - ex.: leg1_joint_name: "rocker_left_joint"
  - ex.: leg2_joint_name: "rocker_right_joint"
  - ex.: leg3_joint_name: "bogie_right_joint"
- leg{$index}_zero_enc_val (int, [0, 0, 0, 0]) - Specifies the encoder value for zero degrees for each {$index}.
  - ex.: leg0_zero_enc_val: 0
  - ex.: leg1_zero_enc_val: 0
  - ex.: leg2_zero_enc_val: 0
  - ex.: leg3_zero_enc_val: 0
- leg0_reverse_dir (bool, false) - Specifies whether to reverse the joint position (enc_val*-1) calculated from the encoder values
  - ex.: leg0_reverse_dir: false
  - ex.: leg1_reverse_dir: false
  - ex.: leg2_reverse_dir: false
  - ex.: leg3_reverse_dir: false
      