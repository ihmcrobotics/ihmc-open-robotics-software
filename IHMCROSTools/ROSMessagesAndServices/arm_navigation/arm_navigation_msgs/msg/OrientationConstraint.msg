# This message contains the definition of an orientation constraint.
Header header

# The robot link this constraint refers to
string link_name

# The type of the constraint
int32 type
int32 LINK_FRAME=0
int32 HEADER_FRAME=1

# The desired orientation of the robot link specified as a quaternion
geometry_msgs/Quaternion orientation

# optional RPY error tolerances specified if 
float64 absolute_roll_tolerance
float64 absolute_pitch_tolerance
float64 absolute_yaw_tolerance

# Constraint weighting factor - a weight for this constraint
float64 weight
