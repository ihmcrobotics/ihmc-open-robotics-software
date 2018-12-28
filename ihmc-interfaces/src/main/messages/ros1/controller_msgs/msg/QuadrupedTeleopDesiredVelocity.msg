# This message is part of the IHMC quadruped controller API.
# This message sends a desired stepping velocity to the step teleop module.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

float64 desired_x_velocity 0
float64 desired_y_velocity 0
float64 desired_yaw_velocity 0