# This message is part of the IHMC behavior module.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Object ID
int32 id

# Position and Orientation of the sensor at the time when the corresponding ImageMessage was published
geometry_msgs/Pose sensor_pose

# Position and Orientation of the object
geometry_msgs/Pose pose

# How confident are we about what object it is. Mostly used to NN based detections
float64 confidence

# Object Category
string object_type

# 2D Vertices of the 3d object bounding box projected onto image plane
geometry_msgs/Point[] bounding_box_2d_vertices

# 3d Vertices of the 3d object Bounding box
geometry_msgs/Point[] bounding_box_vertices

# Optional point cloud for ICP tracked object
geometry_msgs/Point32[] object_point_cloud

geometry_msgs/Point32[] segmented_point_cloud


