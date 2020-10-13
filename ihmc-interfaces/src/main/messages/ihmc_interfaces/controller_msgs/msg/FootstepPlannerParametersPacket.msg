# This message is part of the IHMC footstep planning module.

float64 DEFAULT_NO_VALUE = -11.1
byte ROBOT_SIDE_LEFT = 0
byte ROBOT_SIDE_RIGHT = 1

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Sets whether or not the search should check if the body is colliding with the world. This may cause the planner
# to run slower.
bool check_for_body_box_collisions false

# Enables a collision check that is lighter-weight than a bounding box. Draws a planar region by vertically extruding the line
# between consecutive steps and invalidates steps with collisions, see: ObstacleBetweenNodesChecker
bool check_for_path_collisions false

# Returns the ideal step width for walking on flat ground.
float64 ideal_footstep_width -11.1

# Returns the ideal step length for walking on flat ground.
float64 ideal_footstep_length -11.1

# Returns the ideal step side step width when "shuffling" sideways.
float64 ideal_side_step_width -11.1

# Returns the ideal length when walking backwards. This value is positive.
float64 ideal_back_step_length -11.1

# Returns ideal step length when the vertical height between the start-of-swing and stance feet are at maximum allowed height.
float64 ideal_step_length_at_max_step_z -11.1

# The planner will try to shift footsteps inside of a region so that this value is the minimum distance from the step
# to the edge. A negative value means the footstep can overhang a region.
float64 wiggle_inside_delta_target -11.1

# This parameter only is used if wiggle_while_planning is true. If a step cannot be wiggled inside by this amount or more,
# it will be rejected. Note that if {wiggle_while_planning if false, it's always best effort on the final plan.
float64 wiggle_inside_delta_minimum -11.1

# Maximum xy-distance the planner will consider for candidate steps.
# Step reach refers to the magnitude of the xy-position of a footstep expressed in its parent's z-up sole frame,
# where the parent is the last footstep taken on the other foot.
#
# This parameter is intended to prevent accepting candidate footsteps that are near both the maximum step length and step width.
float64 maximum_step_reach -11.1

# Maximum yaw between consecutive footsteps
#
# A candidate footstep will be rejected if the yaw between it and its parent is greater than this value.
#
# This restricts the planner from planning kinematically infeasible footsteps. It is constant through the
# space of potential steps, so the robot should be able to achieve this yaw, for example, when stepping at
# its maximum reach.
float64 maximum_step_yaw -11.1

# Minimum step width the planner will consider for candidate steps.
#
# Step width refers to the magnitude of the y-position of a footstep expressed in its parent's sole frame,
# where the parent is the last footstep taken on the other foot.
#
# If this value is too low, for example below the foot's width, the planner could place consecutive footsteps
# on top of each other. If too high, footsteps might not be kinematically feasible.
float64 minimum_step_width -11.1

# Minimum step length the planner will consider for candidate steps.
#
# Step length refers to the x-position of a footstep expressed in its parent's sole frame,
# where the parent is the last footstep taken on the other foot.
#
# If this value is too low, for example below the foot's length, the planner could place consecutive footsteps
# on top of each other. If too high, footsteps might not be kinematically feasible.
float64 minimum_step_length -11.1

# Minimum step yaw.
float64 minimum_step_yaw -11.1

# Maximum step reach when stepping up.
#
# Long steps forward are rejected by the planner if one of two criteria are met:
#    The total length of the footstep exceeds {@link #getMaximumStepReachWhenSteppingUp()}, when expressed in its parent's z-up sole frame.
#    - OR -
#    The y-position of the value of the footstep exceeds {@link #getMaximumStepWidthWhenSteppingUp()}, when expressed in its parent's z-up sole frame.
#    - AND -
#    The z-position of the value of the footstep is greater than {@link #getMaximumStepZWhenSteppingUp()}, when expressed in its parent's z-up sole frame.

# Large steps forward and up can cause the robot to surpass its torque limits.
# These parameters should be tuned so that when the robot takes a step of length {@link #getMaximumStepReachWhenSteppingUp()} and {@link #getMaximumStepZWhenSteppingUp()},
# it's very close to saturating its torque limits.
float64 maximum_step_reach_when_stepping_up -11.1

# Maximum step width when stepping up.
#
# Long steps forward are rejected by the planner if one of two criteria are met:
#    The total length of the footstep exceeds {@link #getMaximumStepReachWhenSteppingUp()}, when expressed in its parent's z-up sole frame.
#    - OR -
#    The y-position of the value of the footstep exceeds {@link #getMaximumStepWidthWhenSteppingUp()}, when expressed in its parent's z-up sole frame.
#    - AND -
#    The z-position of the value of the footstep is greater than {@link #getMaximumStepZWhenSteppingUp()}, when expressed in its parent's z-up sole frame.

# Large steps forward and up can cause the robot to surpass its torque limits.
# These parameters should be tuned so that when the robot takes a step of length {@link #getMaximumStepReachWhenSteppingUp()} and {@link #getMaximumStepZWhenSteppingUp()},
# it's very close to saturating its torque limits.
float64 maximum_step_width_when_stepping_up -11.1

# Step height for considering stepping up.
#
# Long steps forward are rejected by the planner if one of two criteria are met:
#    The total length of the footstep exceeds {@link #getMaximumStepReachWhenSteppingUp()}, when expressed in its parent's z-up sole frame.
#    - OR -
#    The y-position of the value of the footstep exceeds {@link #getMaximumStepWidthWhenSteppingUp()}, when expressed in its parent's z-up sole frame.
#    - AND -
#    The z-position of the value of the footstep is greater than {@link #getMaximumStepZWhenSteppingUp()}, when expressed in its parent's z-up sole frame.

# Large steps forward and up can cause the robot to surpass its torque limits.
# These parameters should be tuned so that when the robot takes a step of length {@link #getMaximumStepReachWhenSteppingUp()} and {@link #getMaximumStepZWhenSteppingUp()},
# it's very close to saturating its torque limits.
float64 maximum_step_z_when_stepping_up -11.1

# Maximum step length when stepping forward and down.
#
# Large steps forward and down are rejected by the planner if one of two criteria are met:
#    The x-position of the value of the footstep exceeds maximumStepXWhenForwardAndDown, when expressed in its parent's z-up sole frame
#    - OR -
#    The y-position of the value of the footstep exceeds maximumStepYWhenForwardAndDown, when expressed in its parent's z-up sole frame
#    - AND -
#    The z-position of the value of the footstep is less than maximumStepZWhenForwardAndDown, when expressed in its parent's z-up sole frame
#
# Large steps forward and down usually cause the robot to hit it's ankle pitch joint limit.
# These parameters should be tuned so that when the robot takes a step of length maximumStepXWhenForwardAndDown and height maximumStepZWhenForwardAndDown,
# it's very close to hitting it's ankle pitch joint limit.
float64 maximum_step_x_when_forward_and_down -11.1

# Maximum step width when stepping forward and down.
#
# Large steps forward and down are rejected by the planner if one of two criteria are met:
#    The x-position of the value of the footstep exceeds maximumStepXWhenForwardAndDown, when expressed in its parent's z-up sole frame
#    - OR -
#    The y-position of the value of the footstep exceeds maximumStepYWhenForwardAndDown, when expressed in its parent's z-up sole frame
#    - AND -
#    The z-position of the value of the footstep is less than maximumStepZWhenForwardAndDown, when expressed in its parent's z-up sole frame
#
# Large steps forward and down usually cause the robot to hit it's ankle pitch joint limit.
# These parameters should be tuned so that when the robot takes a step of length maximumStepXWhenForwardAndDown and height maximumStepZWhenForwardAndDown,
# it's very close to hitting it's ankle pitch joint limit.
float64 maximum_step_y_when_forward_and_down -11.1

# Maximum step height when stepping forward and down.
#
# Large steps forward and down are rejected by the planner if one of two criteria are met:
#    The x-position of the value of the footstep exceeds maximumStepXWhenForwardAndDown, when expressed in its parent's z-up sole frame
#    - OR -
#    The y-position of the value of the footstep exceeds maximumStepYWhenForwardAndDown, when expressed in its parent's z-up sole frame
#    - AND -
#    The z-position of the value of the footstep is less than maximumStepZWhenForwardAndDown, when expressed in its parent's z-up sole frame
#
# Large steps forward and down usually cause the robot to hit it's ankle pitch joint limit.
# These parameters should be tuned so that when the robot takes a step of length maximumStepXWhenForwardAndDown and height maximumStepZWhenForwardAndDown,
# it's very close to hitting it's ankle pitch joint limit.
float64 maximum_step_z_when_forward_and_down -11.1

# Maximum vertical distance between consecutive footsteps
#
# A candidate footstep will be rejected if its z-value is greater than this value, when expressed its parent's
# z-up sole frame.
float64 maximum_step_z -11.1

# Maximum vertical distance between start-of-swing and touchdown
float64 maximum_swing_z -11.1

# Maximum xy distance between start-of-swing and touchdown
float64 maximum_swing_reach -11.1

# Maximum vertical distance between consecutive footsteps when the trailing foot is pitched at {@link #getMinimumSurfaceInclineRadians()} .
#
# The maximum depth is determined by linearly interpolating between {@link #getMaximumStepZ()} and this value, based on the fraction the foot is pitched by.
# A candidate footstep will be rejected if its z-value is less than this value, when expressed its parent's z-up sole frame.
float64 minimum_step_z_when_fully_pitched -11.1

# Maximum forward distance between consecutive footsteps when the trailing foot is pitched at {@link #getMinimumSurfaceInclineRadians()} .
#
# The maximum distance is determined by linearly interpolating between {@link #getMaximumStepZ()} and this value, based on the fraction the foot is pitched by.
# A candidate footstep will be rejected if its z-value is less than this value, when expressed its parent's z-up sole frame.
float64 maximum_step_x_when_fully_pitched -11.1

# This is the reduction factor for the max yaw when the step is at max reach.
# This means that, when the footstep is at its maximum distance, this is the fraction reduction of the max yaw.
# If this returns 0.0, the max yaw is not modified, even at full reach.
# If this returns 1.0, the max yaw is 0 at full reach.
#
# That is,
# modifiedMaxYaw = (1.0 - reach / maxReach) * maxYaw + reach / maxReach * (1.0 - alpha) * maxYaw
float64 step_yaw_reduction_factor_at_max_reach -11.1

# Minimum percentage that a candidate footstep needs to overlap with its associated planar region in order to be accepted.
#
# If this parameter is set to 1.0 only full footsteps are allowed. A value less then 1.0 will allow partial footholds.
float64 minimum_foothold_percent 0.9

# The planner will ignore candidate footsteps if they are on a planar region with an incline that is higher
# then the value specified here.
#
# More specifically, if a footstep has an associated planar region and that regions surface normal has a
# z-value less than cos(minimumSurfaceInclineRadians), it will be rejected.
float64 minimum_surface_incline_radians -11.1

# The wiggler can either run as a post-processor on a resulting plan or on each candidate step while planning.
# If true, this wiggles each candidate step, which will slow down plan times but resulting plans will be guarunteed to match step constraints.
bool wiggle_while_planning false

# If wiggle_while_planning is true, this will reject a step if the wiggle meet the specified parameters. If it's false the wiggle does a best effort
bool reject_if_wiggle_not_satisfied false

# There are two solvers for wiggling the step, one constrains to the region's convex hull and the other to the region's concave hull,
# this toggles between them.
bool enable_concave_hull_wiggler false

# When wiggling a candidate footstep into a planar region, this is the maximum distance xy-distance
# distance the planner will use
float64 maximum_xy_wiggle_distance -11.1

# When wiggling a candidate footstep into a planar region, this is the maximum yaw
# distance the planner will use
float64 maximum_yaw_wiggle -11.1

# When snapping a candidate footstep to a planar region, its possible that another planar region
# intersects the footstep at a steep angle, i.e. a valley. If this intersecting planar region
# is never more than maximumZPenetrationOnValleyRegions above the footstep, it won't be rejected,
# otherwise it will.
float64 maximum_z_penetration_on_valley_regions -11.1

# Maximum step width the planner will consider for candidate steps.
#
# Step width refers to the magnitude of the y-position of a footstep expressed in its parent's sole frame,
# where the parent is the last footstep taken on the other foot.
#
# If this value is too low, the planner will unnecessarily reject footsteps. If too high, footsteps might not be kinematically feasible.
float64 maximum_step_width -11.1

# The planner can be setup to avoid footsteps near the bottom of "cliffs". When the footstep has a planar region
# nearby that is cliffHeightToShiftAwayFrom higher than the candidate footstep, it will move away from it
# until it is minimumDistanceFromCliffBottoms away from it.
#
# If these values are set to zero, cliff avoidance will be turned off. This creates a risk that the robot will
# hit the cliff with its swing foot. Therefore, these parameters should be set according to what the swing trajectory
# generator is capable of swinging over.
float64 cliff_base_height_to_avoid -11.1

# The planner can be setup to avoid footsteps near the bottom of "cliffs". When the footstep has a planar region
# nearby that is {@link #getCliffBaseHeightToAvoid} higher than the candidate footstep, it will move away from it
# until it is minimumDistanceFromCliffBottoms away from it.
#
# If these values are set to zero, cliff avoidance will be turned off. This creates a risk that the robot will
# hit the cliff with its swing foot. Therefore, these parameters should be set according to what the swing trajectory
# generator is capable of swinging over.
float64 minimum_distance_from_cliff_bottoms -11.1

# The planner can be setup to avoid footsteps near the top of "cliffs". When the footstep has a planar region
# nearby that is cliffHeightToShiftAwayFrom higher than the candidate footstep, it will move away from it
# until it is minimumDistanceFromCliffTops away from it.
#
# If these values are set to zero, cliff avoidance will be turned off. This creates a risk that the robot will
# hit the cliff with its swing foot. Therefore, these parameters should be set according to what the swing trajectory
# generator is capable of swinging over.
float64 cliff_top_height_to_avoid -11.1

# The planner can be setup to avoid footsteps near the top of "cliffs". When the footstep has a planar region
# nearby that is {@link #getCliffTopHeightToAvoid} higher than the candidate footstep, it will move away from it
# until it is minimumDistanceFromCliffBottoms away from it.
#
# If these values are set to zero, cliff avoidance will be turned off. This creates a risk that the robot will
# hit the cliff with its swing foot. Therefore, these parameters should be set according to what the swing trajectory
# generator is capable of swinging over.
float64 minimum_distance_from_cliff_tops -11.1

# Some node checkers will check if a bounding box that describes the body of the robot will move
# through a planar region (e.g. a wall) when going from one footstep to the next one. To avoid these
# collisions, this defines the box height.
float64 body_box_height -11.1

# Some node checkers will check if a bounding box that describes the body of the robot will move
# through a planar region (e.g. a wall) when going from one footstep to the next one. To avoid these
# collisions, this defines the box depth.
float64 body_box_depth -11.1

# Some node checkers will check if a bounding box that describes the body of the robot will move
# through a planar region (e.g. a wall) when going from one footstep to the next one. To avoid these
# collisions, this defines the box width.
float64 body_box_width -11.1

# Some node checkers will check if a bounding box that describes the body of the robot will move
# through a planar region (e.g. a wall) when going from one footstep to the next one. To avoid these
# collisions, this defines the x-offset of a bounding box relative to the average frame between sequential footsteps.
float64 body_box_base_x -11.1

# Some node checkers will check if a bounding box that describes the body of the robot will move
# through a planar region (e.g. a wall) when going from one footstep to the next one. To avoid these
# collisions, this defines the y-offset of a bounding box relative to the average frame between sequential footsteps.
float64 body_box_base_y -11.1

# Some node checkers will check if a bounding box that describes the body of the robot will move
# through a planar region (e.g. a wall) when going from one footstep to the next one. To avoid these
# collisions, this defines the z-offset of a bounding box relative to the average frame between sequential footsteps.
float64 body_box_base_z -11.1

# Maximum height above a stance step that a candidate step is snapped to. Regions above this height are ignored.
# Intended to avoid ceilings or obstacles that are above the top of the robot
float64 maximum_snap_height -11.1

# Parameter used inside the node expansion to avoid footsteps that would be on top of the stance foot.
# Nodes are only added to the expanded list if they are outside the box around the stance foot defined by
# this parameter.
float64 min_clearance_from_stance -11.1

# Radius around the goal inside which the planner should start to turn to match the goal's orientation
float64 final_turn_proximity -11.1

# When using a cost based planning approach this value defined how the yaw of a footstep will be
# weighted in comparison to its position.
float64 yaw_weight -11.1

# When using a cost based planning approach this value defines how the pitch will be weighted.
float64 pitch_weight -11.1

# When using a cost based planning approach this value defines how the roll will be weighted.
float64 roll_weight -11.1

# When using a cost based planning approach, this value defines how the forward (or backward) displacement
# of a footstep will be weighted in comparison to its position. Note that when using a Euclidean distance, this
# weight is averaged with the value returned by {@link #getLateralWeight()}
float64 forward_weight -11.1

# When using a cost based planning approach, this value defines how the lateral displacement
# of a footstep will be weighted in comparison to its position. Note that when using a Euclidean distance, this
# weight is averaged with the value returned by {@link #getForwardWeight()}
float64 lateral_weight -11.1

# When using a cost based planning approach this value defines how the height change when stepping up will be
# weighted.
float64 step_up_weight -11.1

# When using a cost based planning approach this value defines how the height change when stepping down will be
# weighted.
float64 step_down_weight -11.1

# When using a cost based planning approach this value defines how the a missing foothold area will be weighted.
float64 long_step_weight -11.1

# When using a cost based planning approach this value defines how the a missing foothold area will be weighted.
float64 foothold_area_weight -11.1

# When using a cost based planning approach this value defines the cost that is added for each step
# taken. Setting this value to a high number will favor plans with less steps.
float64 cost_per_step -11.1

# Gets the weight for the heuristics in the A Star planner.
float64 a_star_heuristics_weight -11.1

# This sets how many bounding box checks to perform. If this value is 1, only the final footstep is checked.
# Additional checks are done by interpolating between the start and end steps
uint32 number_of_bounding_box_checks 1

# If this value is non-zero, nodes will be given cost if the bounding box is within this xy distance of a planar region
# @see FootstepPlannerCostParameters#getBoundingBoxCost
float64 maximum_2d_distance_from_bounding_box_to_penalize -11.1

# If the robot's mid-foot pose is within this distance of the body path, it will match the body path heading.
# Otherwise, it will turn towards the body path
float64 distance_from_path_tolerance -11.1

# If the robot's mid-foot pose oriented within this threshold of the body path's heading, it will match the body path heading.
# Otherwise, it will turn in plance towards the body path
float64 delta_yaw_from_reference_tolerance -11.1

# Maximum steps considered at each iteration. If more than this number of steps are available, the closest steps to the
# ideal step are considered and the others are ignored. Set to non-positive number to disable
int32 maximum_branch_factor -1

# If true, enables a mask that reduces the number of calculated steps away from the ideal step
bool enable_expansion_mask true

# If true will try to wiggle steps away from shin collisions. Collisions are checked against all regions.
# Enable concave hull wiggler must be true in order for the shin collision checker to run.
bool enable_shin_collision_check

# How far the shin collision cylinder extends from the toe
float64 shin_toe_clearance -11.1

# How far the shin collision cylinder extends from the heel
float64 shin_heel_clearance -11.1

# Length of the shin collidable cylinder
float64 shin_length -11.1

# Height offset of shin collidable cylinder
float64 shin_height_offet -11.1