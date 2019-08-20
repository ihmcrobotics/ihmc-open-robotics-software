# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Sets whether or not the search should check if the body is colliding with the world. This may cause the planner
# to run slower.
bool check_for_body_box_collisions

# Sets whether or not to perform the defined heuristic search policies.
bool perform_heuristic_search_policies

# Returns the ideal step width for walking on flat ground.
# Field default value -1.0
float64 ideal_footstep_width

# Returns the ideal step length for walking on flat ground.
# Field default value -1.0
float64 ideal_footstep_length

# If the planner in use utilized footstep wiggling (see {@link PolygonWiggler}) to move footholds onto planer
# regions this parameter will be used. It specifies the minimum distance between the foot polygon and the
# edge of the planar region polygon that the footstep is moved into. This value can be negative. That corresponds
# to allowing footsteps that partially intersect planar regions.
# 
# If this value is too high, the planner will not put footsteps on small planar regions. At zero, the planner might
# choose a footstep with an edge along a planar region. This value should roughly be set to the sum of two values:
# The smallest acceptable distance to the edge of a cliff
# The maximum error between desired and actual foot placement
float64 wiggle_inside_delta

# Maximum xy-distance the planner will consider for candidate steps.
# Step reach refers to the magnitude of the xy-position of a footstep expressed in its parent's z-up sole frame,
# where the parent is the last footstep taken on the other foot.
# 
# This parameter is intended to prevent accepting candidate footsteps that are near both the maximum step length and step width.
# Field default value -1.0
float64 maximum_step_reach

# Maximum yaw between consecutive footsteps
# 
# A candidate footstep will be rejected if the yaw between it and its parent is greater than this value.
# 
# This restricts the planner from planning kinematically infeasible footsteps. It is constant through the
# space of potential steps, so the robot should be able to achieve this yaw, for example, when stepping at
# its maximum reach.
# Field default value -1.0
float64 maximum_step_yaw

# Minimum step width the planner will consider for candidate steps.
# 
# Step width refers to the magnitude of the y-position of a footstep expressed in its parent's sole frame,
# where the parent is the last footstep taken on the other foot.
# 
# If this value is too low, for example below the foot's width, the planner could place consecutive footsteps
# on top of each other. If too high, footsteps might not be kinematically feasible.
# Field default value -10.0
float64 minimum_step_width

# Minimum step length the planner will consider for candidate steps.
# 
# Step length refers to the x-position of a footstep expressed in its parent's sole frame,
# where the parent is the last footstep taken on the other foot.
# 
# If this value is too low, for example below the foot's length, the planner could place consecutive footsteps
# on top of each other. If too high, footsteps might not be kinematically feasible.
# Field default value -1.0
float64 minimum_step_length

# Minimum step yaw.
# Field default value -1.0
float64 minimum_step_yaw

# Large steps forward and up can cause the robot to surpass its torque limits.
# These parameters should be tuned so that when the robot takes a step of length {@link #getMaximumStepReachWhenSteppingUp()} and {@link #getMaximumStepZWhenSteppingUp()},
# it's very close to saturating its torque limits.
# Field default value -1.0
float64 maximum_step_reach_when_stepping_up

# Large steps forward and up can cause the robot to surpass its torque limits.
# These parameters should be tuned so that when the robot takes a step of length {@link #getMaximumStepReachWhenSteppingUp()} and {@link #getMaximumStepZWhenSteppingUp()},
# it's very close to saturating its torque limits.
# Field default value -1.0
float64 maximum_step_z_when_stepping_up

# Maximum step length when stepping forward and down.
# 
# Large steps forward and down are rejected by the planner if two criteria are met:
# The x-position of the value of the footstep exceeds maximumStepXWhenForwardAndDown, when expressed in its parent's z-up sole frame
# The z-position of the value of the footstep is less than maximumStepZWhenForwardAndDown, when expressed in its parent's z-up sole frame
# 
# Large steps forward and down usually cause the robot to hit it's ankle pitch joint limit.
# These parameters should be tuned so that when the robot takes a step of length maximumStepXWhenForwardAndDown and height maximumStepZWhenForwardAndDown,
# it's very close to hitting it's ankle pitch joint limit.
# Field default value -1.0
float64 maximum_step_x_when_forward_and_down

# Maximum step height when stepping forward and down.
# 
# Large steps forward and down are rejected by the planner if two criteria are met:
# The x-position of the value of the footstep exceeds maximumStepXWhenForwardAndDown, when expressed in its parent's z-up sole frame
# The z-position of the value of the footstep is less than -maximumStepZWhenForwardAndDown, when expressed in its parent's z-up sole frame
# 
# Large steps forward and down usually cause the robot to hit it's ankle pitch joint limit.
# These parameters should be tuned so that when the robot takes a step of length maximumStepXWhenForwardAndDown and height maximumStepZWhenForwardAndDown,
# it's very close to hitting it's ankle pitch joint limit.
# Field default value -1.0
float64 maximum_step_z_when_forward_and_down

# Maximum vertical distance between consecutive footsteps
# 
# A candidate footstep will be rejected if its z-value is greater than this value, when expressed its parent's
# z-up sole frame.
# Field default value -1.0
float64 maximum_step_z

# Minimum percentage that a candidate footstep needs to overlap with its associated planar region in order to be accepted.
# 
# If this parameter is set to 1.0 only full footsteps are allowed. A value less then 1.0 will allow partial footholds.
# Field default value 0.9
float64 minimum_foothold_percent

# The planner will ignore candidate footsteps if they are on a planar region with an incline that is higher
# then the value specified here.
# 
# More specifically, if a footstep has an associated planar region and that regions surface normal has a
# z-value less than cos(minimumSurfaceInclineRadians), it will be rejected.
# Field default value -1.0
float64 minimum_surface_incline_radians

# There are two methods of wiggling a polygon into a planar region:
# Wiggle the polygon into the planar region itself, which isn't necessarily convex
# Wiggle the polygon into the convex hull of the planar region
# The first method is not implemented completely. Instead it will wiggle into the sub polygon of the planar region that
# has the biggest overlap with the foothold.
# 
# If this parameter is set to true (recommended), the second wiggle method will be used.
# Field default value True
bool wiggle_into_convex_hull_of_planar_regions

# If the planner uses footstep wiggling it attempts to move a candidate footstep inside its associated planar region.
# This attempt is parametrized by {@link #getWiggleIntoConvexHullOfPlanarRegions()}, {@link #getWiggleInsideDelta},
# {@link #getMaximumXYWiggleDistance}, and {@link #getMaximumYawWiggle}. If this transform cannot be found, the
# candidate footstep will be rejected if this method returns {@code true}.
bool reject_if_cannot_fully_wiggle_inside

# When wiggling a candidate footstep into a planar region, this is the maximum distance xy-distance
# distance the planner will use
# Field default value -1.0
float64 maximum_xy_wiggle_distance

# When wiggling a candidate footstep into a planar region, this is the maximum yaw
# distance the planner will use
# Field default value -1.0
float64 maximum_yaw_wiggle

# When snapping a candidate footstep to a planar region, its possible that another planar region
# intersects the footstep at a steep angle, i.e. a valley. If this intersecting planar region
# is never more than maximumZPenetrationOnValleyRegions above the footstep, it won't be rejected,
# otherwise it will.
# Field default value -1.0
float64 maximum_z_penetration_on_valley_regions

# Maximum step width the planner will consider for candidate steps.
# 
# Step width refers to the magnitude of the y-position of a footstep expressed in its parent's sole frame,
# where the parent is the last footstep taken on the other foot.
# 
# If this value is too low, the planner will unnecessarily reject footsteps. If too high, footsteps might not be kinematically feasible.
# Field default value -1.0
float64 maximum_step_width

# The planner can be setup to avoid footsteps near the bottom of "cliffs". When the footstep has a planar region
# nearby that is cliffHeightToShiftAwayFrom higher than the candidate footstep, it will move away from it
# until it is minimumDistanceFromCliffBottoms away from it.
# 
# If these values are set to zero, cliff avoidance will be turned off. This creates a risk that the robot will
# hit the cliff with its swing foot. Therefore, these parameters should be set according to what the swing trajectory
# generator is capable of swinging over.
# Field default value -1.0
float64 cliff_height_to_avoid

# The planner can be setup to avoid footsteps near the bottom of "cliffs". When the footstep has a planar region
# nearby that is {@link #getCliffHeightToAvoid} higher than the candidate footstep, it will move away from it
# until it is minimumDistanceFromCliffBottoms away from it.
# 
# If these values are set to zero, cliff avoidance will be turned off. This creates a risk that the robot will
# hit the cliff with its swing foot. Therefore, these parameters should be set according to what the swing trajectory
# generator is capable of swinging over.
# Field default value -1.0
float64 minimum_distance_from_cliff_bottoms

# When the planner is done planning and cannot find a path to the goal, this flag indicates whether the
# planner should return the best plan that it found. If this value is false, the planner will return
# a {@link FootstepPlan} of type {@link FootstepPlanningResult#NO_PATH_EXISTS}. Otherwise it will return
# "best" is determined by the planner.
bool return_best_effort_plan

# When {@link #getReturnBestEffortPlan()} is true, the planner will return the best effort plan if the plan
# contains at least this many footsteps.
uint32 minimum_steps_for_best_effort_plan

# Some node checkers will check if the body of the robot will move through a higher planar region
# (e.g. a wall) when going from one footstep to the next one. To avoid planar regions close to the
# ground triggering this this parameter defines a ground clearance under which obstacles are allowed.
# This should be set to be slightly above cinder block height (20.3cm) for Atlas.
# Field default value -1.0
float64 body_ground_clearance

# Some node checkers will check if a bounding box that describes the body of the robot will move
# through a planar region (e.g. a wall) when going from one footstep to the next one. To avoid these
# collisions, this defines the box height.
# Field default value -1.0
float64 body_box_height

# Some node checkers will check if a bounding box that describes the body of the robot will move
# through a planar region (e.g. a wall) when going from one footstep to the next one. To avoid these
# collisions, this defines the box depth.
# Field default value -1.0
float64 body_box_depth

# Some node checkers will check if a bounding box that describes the body of the robot will move
# through a planar region (e.g. a wall) when going from one footstep to the next one. To avoid these
# collisions, this defines the box width.
# Field default value -1.0
float64 body_box_width

# Some node checkers will check if a bounding box that describes the body of the robot will move
# through a planar region (e.g. a wall) when going from one footstep to the next one. To avoid these
# collisions, this defines the x-offset of a bounding box relative to the average frame between sequential footsteps.
# Field default value -1.0
float64 body_box_base_x

# Some node checkers will check if a bounding box that describes the body of the robot will move
# through a planar region (e.g. a wall) when going from one footstep to the next one. To avoid these
# collisions, this defines the y-offset of a bounding box relative to the average frame between sequential footsteps.
# Field default value -1.0
float64 body_box_base_y

# Some node checkers will check if a bounding box that describes the body of the robot will move
# through a planar region (e.g. a wall) when going from one footstep to the next one. To avoid these
# collisions, this defines the z-offset of a bounding box relative to the average frame between sequential footsteps.
# Field default value -1.0
float64 body_box_base_z

# Parameter used inside the node expansion to avoid footsteps that would be on top of the stance foot.
# Nodes are only added to the expanded list if they are outside the box around the stance foot defined by
# this parameter.
# Field default value -1.0
float64 min_x_clearance_from_stance

# Parameter used inside the node expansion to avoid footsteps that would be on top of the stance foot.
# Nodes are only added to the expanded list if they are outside the box around the stance foot defined by
# this parameter.
# Field default value -1.0
float64 min_y_clearance_from_stance

# Radius around the goal inside which the planner should start to turn to match the goal's orientation
# Field default value -1.0
float64 final_turn_proximity

# Defines a percentage of the radius around the final turn proximity in which the blending from the desired heading to the
# final orientation should occur. That is, at 1 + {@link #getFinalTurnProximityBlendFactor()}} * {@link #getFinalTurnProximity()},
# the desired orientation is the desired heading, and at 1 - {@link #getFinalTurnProximityBlendFactor()}} * {@link #getFinalTurnProximity()},
# the desired orientation is the final orientation.
# Field default value -1.0
float64 final_turn_proximity_blend_factor

# When using a cost based planning approach this value defined how the yaw of a footstep will be
# weighted in comparison to its position.
# Field default value -1.0
float64 yaw_weight

# When using a cost based planning approach this value defines how the pitch will be weighted.
# Field default value -1.0
float64 pitch_weight

# When using a cost based planning approach this value defines how the roll will be weighted.
# Field default value -1.0
float64 roll_weight

# When using a cost based planning approach, this value defines how the forward (or backward) displacement
# of a footstep will be weighted in comparison to its position. Note that when using a Euclidean distance, this
# weight is averaged with the value returned by {@link #getLateralWeight()}
# Field default value -1.0
float64 forward_weight

# When using a cost based planning approach, this value defines how the lateral displacement
# of a footstep will be weighted in comparison to its position. Note that when using a Euclidean distance, this
# weight is averaged with the value returned by {@link #getForwardWeight()}
# Field default value -1.0
float64 lateral_weight

# When using a cost based planning approach this value defines how the height change when stepping up will be
# weighted.
# Field default value -1.0
float64 step_up_weight

# When using a cost based planning approach this value defines how the height change when stepping down will be
# weighted.
# Field default value -1.0
float64 step_down_weight

# When using a cost based planning approach this value defines how the a missing foothold area will be weighted.
# Field default value -1.0
float64 long_step_weight

# When using a cost based planning approach this value defines how the a missing foothold area will be weighted.
# Field default value -1.0
float64 foothold_area_weight

# When using a cost based planning approach this value defines the cost that is added for each step
# taken. Setting this value to a high number will favor plans with less steps.
# Field default value -1.0
float64 cost_per_step

# Determines which cost function for distance and yaw to use, between {@link QuadraticDistanceAndYawCost} and {@link EuclideanDistanceAndYawBasedCost}
bool use_quadratic_distance_cost

# Determines which cost function for distance and yaw to use, between {@link QuadraticDistanceAndYawCost} and {@link LinearHeightCost}
bool use_quadratic_height_cost

# Gets the weight for the heuristics in the A Star planner.
# Field default value -1.0
float64 a_star_heuristics_weight

# Gets the weight for the heuristics in the Visibility graph with A star planner.
# Field default value -1.0
float64 vis_graph_with_a_star_heuristics_weight

# Gets the weight for the heuristics in the Depth First planner.
# Field default value -1.0
float64 depth_first_heuristics_weight

# Gets the weight for the heuristics in the Body path based planner.
# Field default value -1.0
float64 body_path_based_heuristics_weight

# If this value is non-zero, nodes will be given cost if the bounding box is within this xy distance of a planar region
# @see FootstepPlannerCostParameters#getBoundingBoxCost
# Field default value -1.0
float64 maximum_2d_distance_from_bounding_box_to_penalize

# If a node doesn't have bounding box collisions at the default dimensions, but does when increasing the xy dimensions by d,
# where d < getMaximum2DDistanceFromBoundingBoxToPenalize, there will be a cost given to the node of:
# {@code c * (1 - d / d_max)}, where d_max is this value.
# Field default value -1.0
float64 bounding_box_cost


