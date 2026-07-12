# This message is part of the IHMC footstep planning module.

float64 DEFAULT_NO_VALUE = -11.1

# This message defines the parameters for the A* body path planner.

# Unique ID used to identify this message.
uint64 sequence_id

# Whether or not the planner checks for collisions.
bool check_for_collisions

# Whether the body path plan is post-processed with the smoother.
bool perform_smoothing

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

# Weight placed on a gradient that drives the waypoint towards the initial value.
float64 smoother_displacement_weight

# Gain applied to the smoother gradient for iterative modifications.
float64 smoother_hill_climb_gain

# Minimum gradient vector magnitude to terminate the smoother iterations.
float64 smoother_gradient_threshold_to_terminate

# Distance from the start to perform collision checking. Avoids false-positive collisions of the robot or gantry, for example.
float64 collision_start_tolerance

# Weight on the obstacle clearance cost from the terrain map
float64 obstacle_clearance_cost_weight
