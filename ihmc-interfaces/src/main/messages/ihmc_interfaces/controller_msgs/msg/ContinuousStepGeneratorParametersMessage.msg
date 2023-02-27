# Message for the ContinuousStepGenerator to configure the module's parameters.
# This module is used for controlling the walking gait using a joystick controller.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

int32 number_of_footsteps_to_plan -1
int32 number_of_fixed_footsteps -1
float64 swing_height -1
float64 swing_duration -1
float64 transfer_duration -1
float64 max_step_length -1
float64 default_step_width -1
float64 min_step_width -1
float64 max_step_width -1
float64 turn_max_angle_inward -1
float64 turn_max_angle_outward -1