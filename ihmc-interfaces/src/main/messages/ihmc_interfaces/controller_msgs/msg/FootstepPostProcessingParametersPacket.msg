# This message is part of the IHMC footstep planning module.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Determines whether the post processing module for adjusting the split fractions for the CoP trajectory is enabled.
bool split_fraction_processing_enabled true

# Determines whether the post processing module for swinging over planar regions is enabled.
bool swing_over_regions_enabled true

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

