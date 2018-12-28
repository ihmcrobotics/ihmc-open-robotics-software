# This message is part of the IHMC quadruped controller API.
# This message sends the x gait settings used for determining gait.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

float64 stance_length
float64 stance_width
float64 step_ground_clearance
float64 step_duration
float64 end_double_support_duration
float64 end_phase_shift