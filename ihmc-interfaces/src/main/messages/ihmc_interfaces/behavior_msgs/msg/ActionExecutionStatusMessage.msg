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

# Translation current distance to goal
float64 translation_current_distance_to_goal

# Translation start distance to goal
float64 translation_start_distance_to_goal

# Orientation current distance to goal
float64 orientation_current_distance_to_goal

# Orientation start distance to goal
float64 orientation_start_distance_to_goal