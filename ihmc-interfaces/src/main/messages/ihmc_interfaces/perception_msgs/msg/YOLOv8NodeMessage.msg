# Perception scene node for YOLOv8 detections

# Detectable scene node information
perception_msgs/DetectableSceneNodeMessage detectable_scene_node

# YOLO parameters
float32 detection_acceptance_threshold

# Detection information
string detection_class
float32 confidence
int32 x         # x coordinate of center of detection within image
int32 y         # y coordinate of center of detection within image
int32 width     # width of detection in pixels
int32 height    # height of detection in pixels

geometry_msgs/Point32[<=5000] object_point_cloud
geometry_msgs/Point32 object_centroid
geometry_msgs/Transform centroid_to_object_transform
geometry_msgs/Pose object_pose
geometry_msgs/Pose filtered_object_pose
