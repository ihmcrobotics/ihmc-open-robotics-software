# This message is part of the FootstepStreamingToolbox API.

byte ROBOT_SIDE_LEFT = 0
byte ROBOT_SIDE_RIGHT = 1

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint64 sequence_id

# The timestamp (in nanoseconds) at which this message was generated.
# This is used on the toolbox side to estimate things such as current tracker acceleration.
int64 timestamp 0

# Specifies which foot/tracker
byte side 255
# This is the current position of the robot foot frame's origin.
# The data is assumed to be expressed in world frame.
geometry_msgs/Point robot_foot_position_in_world
# This is the current orientation of the robot foot frame.
# The data is assumed to be expressed in world frame.
geometry_msgs/Quaternion robot_foot_orientation_in_world

# This is the current position of the tracker frame's origin.
# The data is assumed to be expressed in world frame.
geometry_msgs/Point current_position_in_world
# This is the current orientation of the tracker frame.
# The data is assumed to be expressed in world frame.
geometry_msgs/Quaternion current_orientation_in_world

# Whether the current linear velocity is defined.
bool has_current_velocity
# The current linear velocity of the control frame's origin.
geometry_msgs/Vector3 current_linear_velocity_in_world
# The current angular velocity of the control frame.
geometry_msgs/Vector3 current_angular_velocity_in_world


