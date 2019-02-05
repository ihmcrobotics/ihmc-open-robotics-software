# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

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


