# Should disappear for the ROS equivalent.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

int64 timestamp
geometry_msgs/Point sensor_position
geometry_msgs/Quaternion sensor_orientation
# Specifies whether the pointcloud is expressed in the local coordinate system of the sensor or expressed in world frame.
bool is_data_local_to_sensor false

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
# The pointcloud is down to a given resolution defined in meters.
float64 resolution
# The number of points in this frame.
int32 number_of_points
# The pointcloud. The points are stored as 16-bits signed integers.
# To retrieve the actual coordinate use: "(point_cloud.get() + 0.5) * resolution".
# The size of this byte buffer is: "number_of_points * 3 * 2".
# See us.ihmc.robotEnvironmentAwareness.communication.converters.StereoPointCloudCompression for more info on the compression protocol.
byte[<=20000000] point_cloud
# The compressed colors.
# The color for each point in the pointcloud. The colors are stored as 3 bytes: (R, G, B).
# See us.ihmc.robotEnvironmentAwareness.communication.converters.StereoPointCloudCompression for more info on the compression protocol.
byte[<=7000000] colors
# Whether point_cloud and colors have been compressed using the LZ4Compression or not.
bool lz4_compressed