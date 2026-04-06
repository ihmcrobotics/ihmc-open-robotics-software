# Input message for the PDVelocityBasedGoalReacher.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint64 sequence_id

# Goal pose for the robot to reach.
float64 x_position
float64 y_position
float64 yaw

# Goal time to reach the pose. 0.0 means as quickly as possible.
float64 time_to_reach

# Whether to maintain this position after reaching it.
bool hold_position false

# Proximity to the goal.
float64 position_proximity 0.02
float64 orientation_proximity 0.1

# Whether the goal yaw matters. If false, the robot will always face toward the goal
# position rather than blending to match the goal orientation as it approaches.
bool goal_orientation_matters true