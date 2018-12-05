# Message used to encode the manifold of an object to be used as input by a reaching motion planner.
# Main usage is the IHMC WholeBodyTrajectoryToolbox.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

byte CONFIGURATION_SPACE_NAME_X = 0
byte CONFIGURATION_SPACE_NAME_Y = 1
byte CONFIGURATION_SPACE_NAME_Z = 2
byte CONFIGURATION_SPACE_NAME_ROLL = 3
byte CONFIGURATION_SPACE_NAME_PITCH = 4
byte CONFIGURATION_SPACE_NAME_YAW = 5
byte CONFIGURATION_SPACE_NAME_SO3 = 6

int32 end_effector_hash_code
geometry_msgs/Point manifold_origin_position
geometry_msgs/Quaternion manifold_origin_orientation

byte[] manifold_configuration_space_names
float64[] manifold_lower_limits
float64[] manifold_upper_limits