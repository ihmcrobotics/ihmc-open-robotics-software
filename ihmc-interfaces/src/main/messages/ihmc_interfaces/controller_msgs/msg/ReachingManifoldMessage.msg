# Message used to encode the manifold of an object to be used as input by a reaching motion planner.
# Main usage is the IHMC WholeBodyTrajectoryToolbox.

byte CONFIGURATION_SPACE_NAME_X = 0
byte CONFIGURATION_SPACE_NAME_Y = 1
byte CONFIGURATION_SPACE_NAME_Z = 2
byte CONFIGURATION_SPACE_NAME_ROLL = 3
byte CONFIGURATION_SPACE_NAME_PITCH = 4
byte CONFIGURATION_SPACE_NAME_YAW = 5

int64 end_effector_name_based_hash_code
geometry_msgs/Point manifold_origin_position
geometry_msgs/Quaternion manifold_origin_orientation

byte[] manifold_configuration_space_names
float64[] manifold_lower_limits
float64[] manifold_upper_limits