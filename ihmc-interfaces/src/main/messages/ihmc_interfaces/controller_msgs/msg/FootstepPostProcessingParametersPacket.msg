# This message is part of the IHMC footstep planning module.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Determines whether the post processing module for adjusting the split fractions based on the footstep positions for the CoP trajectory is enabled.
bool position_split_fraction_processing_enabled true

# Determines whether the post processing module for adjusting the split fractions based on the foothold areas for the CoP trajectory is enabled.
bool area_split_fraction_processing_enabled true

# Determines whether the post processing module for swinging over planar regions is enabled.
bool swing_over_regions_processing_enabled true

# Sets the step down height for determining whether or not the transfer split fractions should be adjusted.
# If the step height change relative to the stance foot is greater than this value, the split fraction and weight distribution
# will be adjusted so that the CoM is in a more favorable position, kind of "dropping" onto the swing foot.
float64 step_height_for_large_step_down -1.0

# Sets the step down height for the maximum amount of split fraction and weight distribution adjustment.
# If the step height change relative to the stance foot is greater than this value, the split fraction and weight distribution
# will be adjusted fully, as returned by {@link #getTransferSplitFractionAtFullDepth()} and {@link #getTransferWeightDistributionAtFullDepth()}.
float64 largest_step_down_height -1.0

# Sets the desired transfer split fraction if the robot is stepping down by {@link #getLargestStepDownHeight()}.
# If the step down height is between {@link #getStepHeightForLargeStepDown()} and {@link #getLargestStepDownHeight()}, the
# desired split fraction is linearly interpolated between the default value and the value returned by this function.
float64 transfer_split_fraction_at_full_depth -1.0

# Sets the desired transfer weight distribution if the robot is stepping down by {@link #getLargestStepDownHeight()}.
# If the step down height is between {@link #getStepHeightForLargeStepDown()} and {@link #getLargestStepDownHeight()}, the
# desired weight distribution is linearly interpolated between the default value and the value returned by this function.
float64 transfer_weight_distribution_at_full_depth -1.0

bool do_initial_fast_approximation true

# If using the swing over planar regions module, this sets up the minimum swing foot clearance distance between the a ball of radius of the foot length
# along the swing foot trajectory and the planar regions in the environment.
float64 minimum_swing_foot_clearance -1.0

# If using the swing over planar regions module, this is the number of points along the swing foot trajectory that are checked.
uint32 number_of_checks_per_swing 100

# If using the swing over planar regions module, this is the maximum number of iterations for adjusting the swing foot waypoints to attempt avoiding
# collisions with the environment.
uint32 maximum_number_of_adjustment_attempts 50

# If using the swing over planar regions module, this is the maximum adjustment distance of the swing waypoints that will be allowed.
float64 maximum_waypoint_adjustment_distance -1.0

# If using the swing over planar regions module, this is the distance that the swing waypoints will be adjusted by.
float64 incremental_waypoint_adjustment_distance -1.0

float64 minimum_height_above_floor_for_collision -1.0

# If using the area split fraction post processing module, this determines how much of the load a foot should carry during transfer if it has the full
# support area. That is, if the foot has the full area, and we say it should carry the full load, this moves the midpoint CoP position to that foot.
float64 fraction_load_if_foot_has_full_support -1.0

# If using the area split fraction post processing module, this determines how much of the transfer duration should be spent shifting towards the midpoint
# CoP. That is, if the foot has the full area, and we say it should have the entire trajectory (i.e. returns 1), this spends the entire time shifting either
# from the foot to the midpoint, or from the midpoint to that foot.
float64 fraction_time_on_foot_if_foot_has_full_support -1.0

# If using the area split fraction post processing module, this determines how much of the load a foot should carry during transfer if the trailing foot is
# a forward line. That is, if there is only a line contact in the X direction on the other foot, and we say this foot should carry the full load,
# this movies the midpoint CoP position to that foot.
float64 fraction_load_if_other_foot_has_no_width -1.0

# If using the area split fraction post processing module, this determines how much of the transfer duration should be spent shifting towards the midpoint
# CoP. That is, if there is only a line contact in the X direction on the other foot, and we say it should have the entire trajectory (i.e. returns 1),
# this spends the entire time shifting either from the foot to the midpoint, or from the midpoint to that foot.
float64 fraction_time_on_foot_if_other_foot_has_no_width -1.0