# This message is part of the IHMC footstep planning module.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Field default value True
bool do_initial_fast_approximation true

# If using the swing over planar regions module, this sets up the minimum swing foot clearance distance between the a ball of radius of the foot length
# along the swing foot trajectory and the planar regions in the environment.
float64 minimum_swing_foot_clearance -1.0

# If using the swing over planar regions module, this sets up how much less clearance is required on the fast approximation, since it doesn't usually have
# the same amount of curve to the trajectory.
float64 fast_approximation_less_clearance -1.0

# If using the swing over planar regions module, this is the number of points along the swing foot trajectory that are checked.
uint32 number_of_checks_per_swing 100

# If using the swing over planar regions module, this is the maximum number of iterations for adjusting the swing foot waypoints to attempt avoiding
# collisions with the environment.
uint32 maximum_number_of_adjustment_attempts 50

# If using the swing over planar regions module, this is the maximum adjustment distance of the swing waypoints that will be allowed.
float64 maximum_waypoint_adjustment_distance -1.0

# If using the swing over planar regions module, this is the minimum distance that the swing waypoints will be adjusted by on each increment.
float64 minimum_adjustment_increment_distance -1.0

# If using the swing over planar regions module, this is the maximum distance that the swing waypoints will be adjusted by on each increment.
float64 maximum_adjustment_increment_distance -1.0

# If using the swing over planar regions module, this is the scale factor to be applied to the collision on each increment for adjustment.
float64 adjustment_increment_distance_gain -1.0

float64 minimum_height_above_floor_for_collision -1.0

# Collision boxes are checked for the start and end of swing at the toe and heel, respectively.
# If a collision is detected at either, this swing height is used.
float64 swing_height_if_collision_detected -1.0

# Specifies the maximum swing time in the swing waypoint proportion calculator
float64 minimum_swing_time -1.0

# Specifies the maximum swing time in the swing waypoint proportion calculator
float64 maximum_swing_time -1.0

# The waypoint proportion calculator shifts the specified proportions if a foot collision is detected. This specifies the clearance of that collision
float64 foot_stub_clearance -1.0

# The waypoint proportion calculator shifts the specified proportions if a foot collision is detected. This specifies how much to shift if a collision is detected
float64 waypoint_proportion_shift_for_stub_avoidance -1.0

# If using the custom position swing planner, will add additional swing time if the trajectory is expanded
float64 additional_swing_time_if_expanded -1.0