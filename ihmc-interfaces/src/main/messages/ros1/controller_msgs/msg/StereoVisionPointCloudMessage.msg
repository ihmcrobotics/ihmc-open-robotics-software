# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

int64 timestamp

geometry_msgs/Point sensor_position

geometry_msgs/Quaternion sensor_orientation

# There are two types of confidence value noticing the quality of the data for sensor pose and point cloud.
# The range of confidence is from 0.0 with the worst quality to 1.0 with the best quality.
# Field default value 1.0
float64 sensor_pose_confidence

# Field default value 1.0
float64 point_cloud_confidence

float32[] point_cloud

int32[] colors


