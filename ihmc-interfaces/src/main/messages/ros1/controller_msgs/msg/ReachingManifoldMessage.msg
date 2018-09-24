int8 CONFIGURATION_SPACE_NAME_X=0

int8 CONFIGURATION_SPACE_NAME_Y=1

int8 CONFIGURATION_SPACE_NAME_Z=2

int8 CONFIGURATION_SPACE_NAME_ROLL=3

int8 CONFIGURATION_SPACE_NAME_PITCH=4

int8 CONFIGURATION_SPACE_NAME_YAW=5

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

int64 end_effector_name_based_hash_code

geometry_msgs/Point manifold_origin_position

geometry_msgs/Quaternion manifold_origin_orientation

int8[] manifold_configuration_space_names

float64[] manifold_lower_limits

float64[] manifold_upper_limits


