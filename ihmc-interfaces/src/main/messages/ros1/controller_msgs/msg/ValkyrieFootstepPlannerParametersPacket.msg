
# Default float64 field to represent uninitialized value
float64 DEFAULT_NO_VALUE=-11.1

# Returns the ideal step width (i.e. lateral distance) for walking
# Field default value -11.1
float64 ideal_footstep_width

# Returns the farthest backward step length allowed, should be a negative number
# Field default value -11.1
float64 minimum_step_length

# Returns the ideal step length (i.e. forward distance) for walking on flat ground.
# Field default value -11.1
float64 ideal_footstep_length

# Minimum step width allowed.
# Step width is the magnitude of the y-position of a footstep expressed in its parent's (i.e. opposite side) frame.
# Field default value -11.1
float64 minimum_step_width

# Maximum step width allowed.
# Step width is the magnitude of the y-position of a footstep expressed in its parent's (i.e. opposite side) frame.
# Field default value -11.1
float64 maximum_step_width

# Maximum xy distance from the default squared-up stance as specified by ideal_footstep_width
# Field default value -11.1
float64 maximum_step_reach

# Minimum allowed x distance when y distance is less than min_y_clearance_from_stance
# Field default value -11.1
float64 min_x_clearance_from_stance

# Minimum allowed y distance when x distance is less than min_x_clearance_from_stance
# Field default value -11.1
float64 min_y_clearance_from_stance

# Minimum step yaw, i.e. the farthest angle that steps can point towards each other
# Field default value -11.1
float64 minimum_step_yaw

# Maximum step yaw, i.e. the farthest angle that steps can point away from each other
# Field default value -11.1
float64 maximum_step_yaw

# Reduction factor in yaw depending on step reach. At full reach the yaw constraints are scaled by this factor.
# Field default value -11.1
float64 step_yaw_reduction_factor_at_max_reach

# Maximum step height delta allowed, for both step up and step downs
# Field default value -11.1
float64 maximum_step_z

# Minimum percentage that a candidate footstep needs to overlap with its associated planar region in order to be accepted.
# Field default value -11.1
float64 minimum_foothold_percent

# Maximum allowed surface incline to place steps
# Field default value -11.1
float64 maximum_surface_incline_radians

# If true, will wiggle each step while planning. If false, only wiggles the final plan.
# Wiggling while planning slows planner speed by about 50% but is useful when precise foot placements are needed.
bool wiggle_while_planning

# Amount that the planner will attempt to shift footsteps inside of planar regions.
# For example, for a value of 0.01 each footstep will be shifted so that its edge is at least 1cm from the planar region's edge
# Field default value -11.1
float64 wiggle_inside_delta

# Maximum xy distance that steps can be shifted to move into planar regions.
# Field default value -11.1
float64 maximum_xy_wiggle_distance

# Maximum yaw distance that steps can be rotated to move into planar regions.
# Field default value -11.1
float64 maximum_yaw_wiggle

# Height threshold to consider an obstacle a "cliff". Steps will not be closer to cliffs than minimum_distance_from_cliff_bottoms
# Field default value -11.1
float64 cliff_height_to_avoid

# Minimum allowed distance to "cliffs"
# Field default value -11.1
float64 minimum_distance_from_cliff_bottoms

# Steps with height changes below this value will be considered step downs. This value should be negative.
# Field default value -11.1
float64 flat_ground_lower_threshold

# Steps with height changes above this value will be considered step ups
# Field default value -11.1
float64 flat_ground_upper_threshold

# Maximum step with if step height change is lower than flat_ground_lower_threshold
# Field default value -11.1
float64 maximum_step_width_when_stepping_down

# Maximum step reach, i.e. xy distance from nominal stance, when stepping down.
# Field default value -11.1
float64 maximum_step_reach_when_stepping_down

# Maximum step with if the step height exceeds flat_ground_upper_threshold
# Field default value -11.1
float64 maximum_step_width_when_stepping_up

# Maximum step reach, i.e. xy distance from nominal stance, when stepping up.
# Field default value -11.1
float64 maximum_step_reach_when_stepping_up

# Scale factor for checking grandparent node (i.e. previous step on same side).
# If this value is non-zero, if will check the grandparent node on step ups and step downs
# This value represents how much to scale the corresponding step up/down width/reach restrictions.
# This value should be in the range 1.0 - 2.0, recommended to be 1.5.
# Field default value -11.1
float64 translation_scale_from_grandparent_node

# Radius around the goal inside which the planner should start to turn to match the goal's orientation
# Field default value -11.1
float64 final_turn_proximity

# Enables a collision check that is lighter-weight than a bounding box. Draws a planar region by vertically extruding the line
# between consecutive steps and invalidates steps with collisions, see: ObstacleBetweenNodesChecker
bool check_for_path_collisions

# Sets whether or not the search should check if the body is colliding with the world
bool check_for_body_box_collisions

# Body box dimensions
geometry_msgs/Vector3 body_box_dimensions

# Body box offset
geometry_msgs/Vector3 body_box_offset

# Sets the number of body collision checks (minimum is 1).
# Additional checks are done by interpolating between the initial and final steps
# Field default value 1
uint32 number_of_bounding_box_checks

# XYZ translation weight vector
geometry_msgs/Vector3 translation_weight

# Yaw-Pitch-Roll weight vector
geometry_msgs/Vector3 orientation_weight

# Contact cost associated with each step. Increase this value if planner is returning extra steps
# Field default value -11.1
float64 cost_per_step

# Weight for partial foothold cost term
# Field default value -11.1
float64 foothold_area_weight

# Heuristic weight for A* search. Higher heuristic weights will result in quicker plan times,
# but plans might be less optimal.
# Field default value -11.1
float64 a_star_heuristics_weight


