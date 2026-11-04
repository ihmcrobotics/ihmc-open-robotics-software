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

# When true, keep the current heading all the way to the goal (walk backward or
# sideways even past distanceToFaceGoal). Default false faces the goal when far.
bool keep_heading false

# When true, turn in place to face the goal, walk straight to it, then turn in place to the goal
# yaw. Takes precedence over keep_heading.
bool turn_walk_turn false

# Walking speed limit in meters per second for this goal. The backward and lateral limits scale with it.
# 0 or less uses the controller limit.
float64 max_forward_speed 0.0

# Turning speed limit in radians per second for this goal. 0 or less uses the controller limit.
float64 max_turning_speed 0.0
