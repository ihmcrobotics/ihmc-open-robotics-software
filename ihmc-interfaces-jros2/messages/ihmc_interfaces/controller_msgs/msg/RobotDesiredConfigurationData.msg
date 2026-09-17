# This message is part of the IHMC whole-body controller API.
# This message contains a list of joint-level desired values that are output from the whole-body controller

# Wall-time in nanoseconds
int64 wall_time

# Hash of the joint names included in joint_desired_output_list
int32 joint_name_hash

# Contains list of joint desireds, ordered according
JointDesiredOutputMessage[<=50] joint_desired_output_list

bool has_desired_root_joint_position_data
geometry_msgs/Vector3 desired_root_joint_translation
geometry_msgs/Quaternion desired_root_joint_orientation

bool has_desired_root_joint_velocity_data
geometry_msgs/Vector3 desired_root_joint_linear_velocity
geometry_msgs/Vector3 desired_root_joint_angular_velocity

bool has_desired_root_joint_acceleration_data
geometry_msgs/Vector3 desired_root_joint_linear_acceleration
geometry_msgs/Vector3 desired_root_joint_angular_acceleration
