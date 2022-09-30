# This message is part of the IHMC quadruped controller API.
# This message sends the x gait settings used for determining gait.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

float64 step_duration -1.0
float64 end_double_support_duration -1.0
float64 max_speed -1.0