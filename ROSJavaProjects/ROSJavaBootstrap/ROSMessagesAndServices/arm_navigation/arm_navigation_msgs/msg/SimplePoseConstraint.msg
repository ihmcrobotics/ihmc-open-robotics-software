# This message contains the definition of a simple pose constraint 
# that specifies the pose for a particular link of the robot and corresponding
# (absolute) position and orientation tolerances

# The standard ROS message header
Header header

# The robot link this constraint refers to
string link_name

# The desired position of the robot link
geometry_msgs/Pose pose

# Position (absolute) tolerance
geometry_msgs/Point absolute_position_tolerance

# Orientation (absolute) tolerance
float64 absolute_roll_tolerance
float64 absolute_yaw_tolerance
float64 absolute_pitch_tolerance

int32 orientation_constraint_type
int32 HEADER_FRAME=0
int32 LINK_FRAME=1
