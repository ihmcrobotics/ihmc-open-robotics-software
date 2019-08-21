int8 SLOW=0

int8 MEDIUM=1

int8 FAST=2

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Field default value -1.0
float64 end_phase_shift

# Field default value -1.0
float64 stance_length

# Field default value -1.0
float64 stance_width

# Field default value -1.0
float64 step_ground_clearance

# Field default value -1.0
float64 max_horizontal_speed_fraction

# Field default value -1.0
float64 max_yaw_speed_fraction

int8 quadruped_speed

controller_msgs/QuadrupedGaitTimingsPacket pace_slow_settings_packet

controller_msgs/QuadrupedGaitTimingsPacket pace_medium_settings_packet

controller_msgs/QuadrupedGaitTimingsPacket pace_fast_settings_packet

controller_msgs/QuadrupedGaitTimingsPacket amble_slow_settings_packet

controller_msgs/QuadrupedGaitTimingsPacket amble_medium_settings_packet

controller_msgs/QuadrupedGaitTimingsPacket amble_fast_settings_packet

controller_msgs/QuadrupedGaitTimingsPacket trot_slow_settings_packet

controller_msgs/QuadrupedGaitTimingsPacket trot_medium_settings_packet

controller_msgs/QuadrupedGaitTimingsPacket trot_fast_settings_packet


