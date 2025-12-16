# Parent definition fields
behavior_msgs/ActionNodeDefinitionMessage definition

# YOLO model name to use
string yolo_model_name

# YOLO confidence threshold
float32 yolo_confidence_threshold

# YOLO mask threshold
float32 yolo_mask_threshold

# Segmentation mask erosion radius
uint16 segmentation_mask_erosion_radius

# Threshold for removing outlier points of segmented point cloud
float32 outlier_threshold

# FoundationPose object type (enum ordinal)
byte object_type

# True to use foundation pose, else use a YOLO based detection
bool use_foundation_pose

# YOLO models that should be enabled at this point
string[<=10] enabled_yolo_models

# YOLO classes to ignore
byte[<=256] ignored_yolo_class_indices

# FoundationPose models that should be enabled at this point
byte[<=10] enabled_foundation_pose_models
