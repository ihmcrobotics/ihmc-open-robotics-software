# Input message for the PDVelocityBasedGoalReacher.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint64 sequence_id

# Goal pose for the robot to reach.
float64 x_position
float64 y_position
float64 yaw

# Goal time to reach the pose. 0.0 means as quickly as possible.
float64 time_to_reach
