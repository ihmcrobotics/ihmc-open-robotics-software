# Should disappear for the ROS equivalent.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

int64 timestamp
geometry_msgs/Point sensor_position
geometry_msgs/Quaternion sensor_orientation

# There are two types of confidence value noticing the quality of the data for sensor pose and point cloud.
# The range of confidence is from 0.0 with the worst quality to 1.0 with the best quality.
# The confidence of the sensor pose represents the quality of the pose estimation.
float64 sensor_pose_confidence 1.0
# There are two types of confidence value noticing the quality of the data for sensor pose and point cloud.
# The range of confidence is from 0.0 with the worst quality to 1.0 with the best quality.
# The confidence of the point cloud represents the quality of the collected point cloud data.
float64 point_cloud_confidence 1.0

# The center location of the bounding box of all the points.
# The location of each point in the pointcloud is with respect to this location.
geometry_msgs/Point point_cloud_center
# The pointcloud is compressed by using an octree.
# This indicates the resolution used for the octree, the octree depth is 16.
float64 resolution
# The number of points in this frame.
int32 number_of_points
# The compressed pointcloud.
# See us.ihmc.robotEnvironmentAwareness.communication.converters.PointCloudCompression for more info on the compression protocol.
byte[<=2000000] point_cloud
# The compressed colors.
# See us.ihmc.robotEnvironmentAwareness.communication.converters.PointCloudCompression for more info on the compression protocol.
byte[<=700000] colors