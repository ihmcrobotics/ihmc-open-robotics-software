# This message should disappear to be replaced by ROS equivalent.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

int64 robot_timestamp

geometry_msgs/Point lidar_position

geometry_msgs/Quaternion lidar_orientation

# There are two types of confidence value noticing the quality of the data for sensor pose and point cloud.
# The range of confidence is from 0.0 with the worst quality to 1.0 with the best quality.
# The confidence of the sensor pose represents the quality of the pose estimation.
# Field default value 1.0
float64 sensor_pose_confidence

# There are two types of confidence value noticing the quality of the data for sensor pose and point cloud.
# The range of confidence is from 0.0 with the worst quality to 1.0 with the best quality.
# The confidence of the point cloud represents the quality of the collected point cloud data.
# Field default value 1.0
float64 point_cloud_confidence

# Number of points in the pointcloud
int32 number_of_points

# Compressed pointcloud data. See us.ihmc.communication.packets.LidarPointCloudCompression for method to compress and decompress
int8[] scan


