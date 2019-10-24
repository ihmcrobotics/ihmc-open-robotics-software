# This message should disappear to be replaced by ROS equivalent.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

int64 robot_timestamp
geometry_msgs/Point lidar_position
geometry_msgs/Quaternion lidar_orientation

# There are two types of confidence value noticing the quality of the data for sensor pose and point cloud.
# The range of confidence is from 0.0 with the worst quality to 1.0 with the best quality.
float64 sensor_pose_confidence 1.0
float64 point_cloud_confidence 1.0
float32[<=200000] scan
