# Published by the PDVelocityBasedGoalReacher when it begins walking toward a waypoint
# or when it reaches a waypoint.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint64 sequence_id

# The waypoint goal pose associated with this event.
float64 goal_x_position
float64 goal_y_position
float64 goal_yaw

# Number of waypoints remaining in the queue after this event (not counting the waypoint
# described by this message).
int32 waypoints_remaining

# True if the robot just started walking toward this waypoint.
# False if the robot just reached this waypoint.
bool is_started false
