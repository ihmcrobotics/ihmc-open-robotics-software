# This message is part of the IHMC footstep planning module.
# The parameters below are used in the class CollisionFreeSwingCalculator, which computes a trajectory with ~10 knot points.
# The naming below is used to describe a trapezoid curve, such that parameters have a low and high value, and are interpolated
# between them based on a low and high percentage through the trajectory.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Field default value True
# Field default value True
bool do_initial_fast_approximation

# If using the swing over planar regions module, this sets up the minimum swing foot clearance distance between the a ball of radius of the foot length
# along the swing foot trajectory and the planar regions in the environment.
# Field default value -1.0
float64 minimum_swing_foot_clearance

# If using the swing over planar regions module, this sets up how much less clearance is required on the fast approximation, since it doesn't usually have
# the same amount of curve to the trajectory.
# Field default value -1.0
float64 fast_approximation_less_clearance

# If using the swing over planar regions module, this is the number of points along the swing foot trajectory that are checked.
# Field default value 100
uint32 number_of_checks_per_swing

# If using the swing over planar regions module, this is the maximum number of iterations for adjusting the swing foot waypoints to attempt avoiding
# collisions with the environment.
# Field default value 50
uint32 maximum_number_of_adjustment_attempts

# If using the swing over planar regions module, this is the maximum adjustment distance of the swing waypoints that will be allowed.
# Field default value -1.0
float64 maximum_waypoint_adjustment_distance

# If using the swing over planar regions module, this is the minimum distance that the swing waypoints will be adjusted by on each increment.
# Field default value -1.0
float64 minimum_adjustment_increment_distance

# If using the swing over planar regions module, this is the maximum distance that the swing waypoints will be adjusted by on each increment.
# Field default value -1.0
float64 maximum_adjustment_increment_distance

# If using the swing over planar regions module, this is the scale factor to be applied to the collision on each increment for adjustment.
# Field default value -1.0
float64 adjustment_increment_distance_gain

# Field default value -1.0
float64 minimum_height_above_floor_for_collision

# Collision boxes are checked for the start and end of swing at the toe and heel, respectively.
# If a collision is detected at either, this swing height is used.
# Field default value -1.0
float64 swing_height_if_collision_detected

# Specifies the maximum swing time in the swing waypoint proportion calculator
# Field default value -1.0
float64 minimum_swing_time

# Specifies the maximum swing time in the swing waypoint proportion calculator
# Field default value -1.0
float64 maximum_swing_time

# The waypoint proportion calculator shifts the specified proportions if a foot collision is detected. This specifies the clearance of that collision
# Field default value -1.0
float64 foot_stub_clearance

# The waypoint proportion calculator shifts the specified proportions if a foot collision is detected. This specifies how much to shift if a collision is detected
# Field default value -1.0
float64 waypoint_proportion_shift_for_stub_avoidance

# If using the custom position swing planner, will add additional swing time if the trajectory is expanded
# Field default value -1.0
float64 additional_swing_time_if_expanded

# If true, the swing plan allows lateral relative to the nominal trajectory.
# Field default value True
bool allow_lateral_motion

# If a footstep has less than this xy displacement during swing, no swing trajectory is planned
# Field default value -1.0
float64 min_xy_translation_to_plan_swing

# Field default value -1.0
float64 percentage_extra_size_x_low

# Field default value -1.0
float64 percentage_extra_size_x_high

# Field default value -1.0
float64 extra_size_x_low

# Field default value -1.0
float64 extra_size_x_high

# Field default value -1.0
float64 percentage_extra_size_y_low

# Field default value -1.0
float64 percentage_extra_size_y_high

# Field default value -1.0
float64 extra_size_y_low

# Field default value -1.0
float64 extra_size_y_high

# Field default value -1.0
float64 percentage_extra_size_z_low

# Field default value -1.0
float64 percentage_extra_size_z_high

# Field default value -1.0
float64 extra_size_z_low

# Field default value -1.0
float64 extra_size_z_high

# Field default value -1.0
float64 percentage_max_displacement_low

# Field default value -1.0
float64 percentage_max_displacement_high

# Field default value -1.0
float64 max_displacement_low

# Field default value -1.0
float64 max_displacement_high

# Field default value -1.0
float64 motion_correlation_alpha


