std_msgs/Header header

# debug
int64 dropped_messages

int64 sensor_head_pps_timestamp
int32 joint_name_hash

float32[<=50] joint_angles
float32[<=50] joint_velocities
float32[<=50] joint_torques
geometry_msgs/Wrench[<=50] force_sensor_data

sensor_msgs/Imu[<=5] imu_sensor_data
geometry_msgs/Vector3 root_translation
geometry_msgs/Quaternion root_orientation
geometry_msgs/Vector3 pelvis_linear_velocity
geometry_msgs/Vector3 pelvis_angular_velocity
geometry_msgs/Vector3 pelvis_linear_acceleration

int32 STANDING = 3
int32 IN_MOTION = 4
int32 robot_motion_status

# For verifying the robot is receiving your commands
int32 last_received_packet_type_id
int64 last_received_packet_unique_id
int64 last_received_packet_robot_timestamp
