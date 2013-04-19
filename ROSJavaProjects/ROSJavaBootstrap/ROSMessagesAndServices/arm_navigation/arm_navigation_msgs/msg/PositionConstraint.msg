# This message contains the definition of a position constraint.
Header header

# The robot link this constraint refers to
string link_name

# The offset (in the link frame) for the target point on the link we are planning for
geometry_msgs/Point target_point_offset

# The nominal/target position for the point we are planning for
geometry_msgs/Point position

# The shape of the bounded region that constrains the position of the end-effector
# This region is always centered at the position defined above
arm_navigation_msgs/Shape constraint_region_shape

# The orientation of the bounded region that constrains the position of the end-effector. 
# This allows the specification of non-axis aligned constraints
geometry_msgs/Quaternion constraint_region_orientation

# Constraint weighting factor - a weight for this constraint
float64 weight
