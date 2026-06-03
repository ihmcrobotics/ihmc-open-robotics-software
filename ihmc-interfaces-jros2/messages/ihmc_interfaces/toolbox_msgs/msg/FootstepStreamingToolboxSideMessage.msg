# This message contains all the information needed to estimate footsteps for the robot based on VR trackers

byte ROBOT_SIDE_LEFT = 0
byte ROBOT_SIDE_RIGHT = 1

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint64 sequence_id

# Specifies which foot/tracker
byte side 255

# The timestamp of the  last read from the tracker
int64 timestamp 0

# Current position of the robot foot frame's origin.
# The data is assumed to be expressed in world frame.
geometry_msgs/Point robot_foot_position_in_world
# Current orientation of the robot foot frame.
# The data is assumed to be expressed in world frame.
geometry_msgs/Quaternion robot_foot_orientation_in_world

# Current position of the tracker frame's origin.
# The data is assumed to be expressed in world frame.
geometry_msgs/Point current_position_in_world
# Current orientation of the tracker frame.
# The data is assumed to be expressed in world frame.
geometry_msgs/Quaternion current_orientation_in_world

# Whether the current linear velocity is defined.
bool has_current_velocity
# The current linear velocity of the tracker frame's origin.
geometry_msgs/Vector3 current_linear_velocity_in_world
# The current angular velocity of the tracker frame.
geometry_msgs/Vector3 current_angular_velocity_in_world