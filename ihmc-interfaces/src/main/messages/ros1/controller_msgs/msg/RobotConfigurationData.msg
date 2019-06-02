int8 ROBOT_MOTION_STATUS_UNKNOWN=0

int8 ROBOT_MOTION_STATUS_STANDING=1

int8 ROBOT_MOTION_STATUS_IN_MOTION=2

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Absolute timestamp (UTC). This is usually used with ROS and is different from the controller up-time and machine up-time.
int64 timestamp

# Timestamp representing the controller up-time.
int64 controller_timestamp

int64 sensor_head_pps_timestamp

int32 joint_name_hash

float32[] joint_angles

float32[] joint_velocities

float32[] joint_torques

geometry_msgs/Vector3 root_translation

geometry_msgs/Quaternion root_orientation

geometry_msgs/Vector3 pelvis_linear_velocity

geometry_msgs/Vector3 pelvis_angular_velocity

geometry_msgs/Vector3 pelvis_linear_acceleration

controller_msgs/SpatialVectorMessage[] force_sensor_data

controller_msgs/IMUPacket[] imu_sensor_data

# Field default value 255
int8 robot_motion_status

int32 last_received_packet_type_id

int64 last_received_packet_unique_id

int64 last_received_packet_robot_timestamp


