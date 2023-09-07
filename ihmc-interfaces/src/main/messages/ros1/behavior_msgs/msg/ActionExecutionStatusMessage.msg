# This message is meant to communicate the status of currently executing actions

# Executing action index
int32 action_index

# Nominal execution duration
float64 nominal_execution_duration

# Time since execution started
float64 elapsed_execution_time

# Total number of footsteps; used for walking actions
uint16 total_number_of_footsteps

# Incomplete footsteps; used for walking actions
uint16 number_of_incomplete_footsteps

# Current position distance to goal
float64 current_position_distance_to_goal

# Start position distance to goal
float64 start_position_distance_to_goal

# Position distance to goal tolerance
float64 position_distance_to_goal_tolerance

# Current orientation distance to goal
float64 current_orientation_distance_to_goal

# Start orientation distance to goal
float64 start_orientation_distance_to_goal

# Orientation distance to goal tolerance
float64 orientation_distance_to_goal_tolerance

# Linear hand wrench magnitude
float64 hand_wrench_magnitude_linear


