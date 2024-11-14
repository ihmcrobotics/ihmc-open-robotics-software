# This message is part of the IHMC footstep planning module.

float64 DEFAULT_NO_VALUE = -11.1

# This message defines the parameters for the A* body path planner.

# Unique ID used to identify this message.
uint32 sequence_id

# Whether or not the planner checks for collisions.
bool check_for_collisions

# Whether the planner computes surface normals. If false, traversibility and roll are also not computed.
bool compute_surface_normal_cost

# Whether the planner computes and checks traversibility.
bool compute_traversibility

# Whether the body path plan is post-processed with the smoother.
bool perform_smoothing

# This is the weight assigned to roll in the search space. Increasing this value will decrease the likelihood of the path moving sideways across slopes.
float64 roll_cost_weight

# The angle in degrees of the deadband applied to the computed roll. Increasing this value decreases the effect small rolls have on the path.
float64 roll_cost_deadband

# When the roll is below this angle, the cost is linearly discounted to zero. Increasing this value decreases the effect roll has on the path.
float64 max_penalized_roll_angle

# The node height of a vertex is determined as the average height of all cells within this radius.
float64 snap_radius

# When computing the vertex height, cells that are this distance below the max height are ignored when taking the average.
float64 min_snap_height_threshold

# Weight assigned to minimizing the incline the path takes.
float64 incline_cost_weight

# Deadband applied to the incline in the search.
float64 incline_cost_deadband

# The maximum incline, in degrees, that is allowed for the body path planner to traverse.
float64 max_incline

# Width of the collision box used for checking collisions with the environment.
float64 collision_box_size_y

# Depth of the collision box used for checking collisions with the environment.
float64 collision_box_size_x

# Intersection height below which collisions are ignored.
float64 collision_box_ground_clearance

# Weight placed on maximizing traversibility in the plan.
float64 traversibility_weight

# Weight placed on the traversibility at the start node when computing the overall traversibility score.
float64 traversibility_stance_weight

# Weight placed on the traversibility at the end node when computing the overall traversibility score.
float64 traversibility_step_weight

# Minimum score on the start node traversibility to say an edge is traversible.
float64 min_traversibility_score

# Minimum angle in degrees to penalize in the traversibility score.
float64 min_normal_angle_to_penalize_for_traversibility

# Maximum angle in degrees to penalize in the traversibility score.
float64 max_normal_angle_to_penalize_for_traversibility

# Weight to place on the surface normals when computing the traversibility score.
float64 traversibility_incline_weight

# Box width of cells to include when performing the traversibility calculation.
float64 traversibility_search_width

# Minimum number of occupied neighbor cells to say a vertex is traversible.
int32 min_occupied_neighbors_for_traversibility

# Half the typical stance width of the robot, used to compute the traversibility.
float64 half_stance_width

# Width of the height window of cells to include in the traversibility calculation.
float64 traversibility_height_window_width

# Deadband applied to the height distance of cells in the traversibility calculation.
float64 traversibility_height_window_deadband

# Distance to the ground plane estimate that is needed to say the two nodes are in the ground plane.
float64 height_proximity_for_saying_walking_on_ground

# Minimum discount applied to non-ground cells when computing the traversibility on the ground plane.
float64 traversibility_non_ground_discount_when_walking_on_ground

# Weight placed on the gradient for avoiding collisions in the smoother.
float64 smoother_collision_weight

# Weight placed on the gradient for minimizing the angle between successive segments of the body path.
float64 smoother_smoothness_weight

# Discount applied to the smoothness gradients of turn points.
float64 smoother_turn_point_smoothness_discount

# Minimum curvature in degrees to penalize with a gradient.
float64 smoother_min_curvature_to_penalize

# Weight placed on the gradient for making vertices of the body path plan an equal distance apart.
float64 smoother_equal_spacing_weight

# Weight placed on the gradient for minimizing the roll cost of the body path plan.
float64 smoother_roll_weight

# Weight placed on a gradient that drives the waypoint towards the initial value.
float64 smoother_displacement_weight

# Weight placed on the gradient for maximizing the traversibility of the body path plan.
float64 smoother_traversibility_weight

# Weight placed on the gradient pushing the body path plan towards cells in the ground plane.
float64 smoother_ground_plane_weight

# Traversibility threshold resulting in a zero gradient for traversibility.
float64 smoother_minimum_traversibility_to_search_for

# Traversibility threshold above which the traversibility gradient begins to carry less weight.
float64 smoother_traversibility_threshold_for_no_discount

# Gain applied to the smoother gradient for iterative modifications.
float64 smoother_hill_climb_gain

# Minimum gradient vector magnitude to terminate the smoother iterations.
float64 smoother_gradient_threshold_to_terminate