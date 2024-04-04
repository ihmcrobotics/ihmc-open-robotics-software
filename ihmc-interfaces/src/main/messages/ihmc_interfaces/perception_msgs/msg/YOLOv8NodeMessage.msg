# Perception scene node for YOLOv8 detections

# Detectable scene node information
perception_msgs/DetectableSceneNodeMessage detectable_scene_node

# YOLO parameters
int32 mask_erosion_kernel_radius
float64 outlier_filter_threshold
float32 detection_acceptance_threshold

# Detection formation
string detection_class
geometry_msgs/Point32[<=5000] object_point_cloud
geometry_msgs/Point32 object_centroid
geometry_msgs/Transform centroid_to_object_transform
geometry_msgs/Pose object_pose
geometry_msgs/Pose filtered_object_pose

# The visual transform to parent
geometry_msgs/Transform visual_transform_to_object_pose

# Alpha filter for transform interpolation
float32 alpha_filter