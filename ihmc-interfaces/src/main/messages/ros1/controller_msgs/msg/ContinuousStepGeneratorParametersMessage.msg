# Message for the ContinuousStepGenerator to configure the module's parameters.
# This module is used for controlling the walking gait using a joystick controller.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Field default value -1
int32 number_of_footsteps_to_plan

# Field default value -1
int32 number_of_fixed_footsteps

# Field default value -1.0
float64 swing_height

# Field default value -1.0
float64 swing_duration

# Field default value -1.0
float64 transfer_duration

# Field default value -1.0
float64 max_step_length

# Field default value -1.0
float64 default_step_width

# Field default value -1.0
float64 min_step_width

# Field default value -1.0
float64 max_step_width

# Field default value -1.0
float64 turn_max_angle_inward

# Field default value -1.0
float64 turn_max_angle_outward


