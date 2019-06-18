# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

int32 joint_name_hash

float32[] desired_joint_angles

geometry_msgs/Vector3 desired_root_translation

geometry_msgs/Quaternion desired_root_orientation

# Desired joint velocities might be empty.
float32[] desired_joint_velocities

# Desired twist of root might be empty.
geometry_msgs/Vector3 desired_root_linear_velocity

geometry_msgs/Vector3 desired_root_angular_velocity

# Field default value -1.0
float64 solution_quality


