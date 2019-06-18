# This message should disappear to be replaced by ROS equivalent.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

int64 robot_timestamp
geometry_msgs/Point lidar_position
geometry_msgs/Quaternion lidar_orientation
float32[<=200000] scan
