# This message is part of the IHMC whole-body controller API.
# This message commands the controller to brace against the given planar region

byte ROBOT_SIDE_LEFT = 0
byte ROBOT_SIDE_RIGHT = 1

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint64 sequence_id

# Specifies which hand will execute the trajectory.
byte robot_side 255

# Trajectory duration in seconds
float64 trajectory_duration 0.0

# Specifies the desired bracing point
geometry_msgs/Point bracing_point

# Specifies the normal vector of the bracing surface
geometry_msgs/Vector3 bracing_normal
