# Perception scene node for YOLOv8 ICP detection

# Detectable scene node information
perception_msgs/DetectableSceneNodeMessage detectable_scene_node

# YOLO/ICP Parameters
int32 mask_erosion_kernel_radius
float64 outlier_filter_threshold
int32 icp_iterations
float64 base_distance_threshold
bool run_icp

# Status
float64 movement_distance_threshold
float64 detection_frequency