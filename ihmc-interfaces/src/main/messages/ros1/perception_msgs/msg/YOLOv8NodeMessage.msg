# Perception scene node for YOLOv8 detections

# Detectable scene node information
perception_msgs/DetectableSceneNodeMessage detectable_scene_node

float64 confidence

geometry_msgs/Point32[] object_point_cloud

# YOLOv8Node data
geometry_msgs/Transform centroid_to_object_transform

geometry_msgs/Pose object_pose

geometry_msgs/Pose filtered_object_pose


