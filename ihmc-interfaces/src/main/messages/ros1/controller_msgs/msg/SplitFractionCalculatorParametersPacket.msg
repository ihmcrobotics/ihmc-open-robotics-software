# This message is part of the IHMC footstep planning module.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Default value for the ICP plan's transfer split fraction.
# Field default value -1.0
float64 default_transfer_split_fraction

# Sets the step down height for determining whether or not the transfer split fractions should be adjusted.
# If the step height change relative to the stance foot is greater than this value, the split fraction and weight distribution
# will be adjusted so that the CoM is in a more favorable position, kind of "dropping" onto the swing foot.
# Field default value -1.0
float64 step_height_for_large_step_down

# Sets the step down height for the maximum amount of split fraction and weight distribution adjustment.
# If the step height change relative to the stance foot is greater than this value, the split fraction and weight distribution
# will be adjusted fully, as returned by {@link #getTransferSplitFractionAtFullDepth()} and {@link #getTransferWeightDistributionAtFullDepth()}.
# Field default value -1.0
float64 largest_step_down_height

# Sets the desired transfer split fraction if the robot is stepping down by {@link #getLargestStepDownHeight()}.
# If the step down height is between {@link #getStepHeightForLargeStepDown()} and {@link #getLargestStepDownHeight()}, the
# desired split fraction is linearly interpolated between the default value and the value returned by this function.
# Field default value -1.0
float64 transfer_split_fraction_at_full_depth

# Sets the desired transfer weight distribution if the robot is stepping down by {@link #getLargestStepDownHeight()}.
# If the step down height is between {@link #getStepHeightForLargeStepDown()} and {@link #getLargestStepDownHeight()}, the
# desired weight distribution is linearly interpolated between the default value and the value returned by this function.
# Field default value -1.0
float64 transfer_weight_distribution_at_full_depth

# If using the area split fraction post processing module, this determines how much of the load a foot should carry during transfer if it has the full
# support area. That is, if the foot has the full area, and we say it should carry the full load, this moves the midpoint CoP position to that foot.
# Field default value -1.0
float64 fraction_load_if_foot_has_full_support

# If using the area split fraction post processing module, this determines how much of the transfer duration should be spent shifting towards the midpoint
# CoP. That is, if the foot has the full area, and we say it should have the entire trajectory (i.e. returns 1), this spends the entire time shifting either
# from the foot to the midpoint, or from the midpoint to that foot.
# Field default value -1.0
float64 fraction_time_on_foot_if_foot_has_full_support

# If using the area split fraction post processing module, this determines how much of the load a foot should carry during transfer if the trailing foot is
# a forward line. That is, if there is only a line contact in the X direction on the other foot, and we say this foot should carry the full load,
# this movies the midpoint CoP position to that foot.
# Field default value -1.0
float64 fraction_load_if_other_foot_has_no_width

# If using the area split fraction post processing module, this determines how much of the transfer duration should be spent shifting towards the midpoint
# CoP. That is, if there is only a line contact in the X direction on the other foot, and we say it should have the entire trajectory (i.e. returns 1),
# this spends the entire time shifting either from the foot to the midpoint, or from the midpoint to that foot.
# Field default value -1.0
float64 fraction_time_on_foot_if_other_foot_has_no_width


