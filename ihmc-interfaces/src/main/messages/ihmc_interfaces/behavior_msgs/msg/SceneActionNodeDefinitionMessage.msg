# Parent definition fields
behavior_msgs/ActionNodeDefinitionMessage definition

# TODO: Add other actions besides setting up an object

# Definition of the object to setup
behavior_msgs/BehaviorTreeSceneObjectDefinitionMessage scene_object_definition

# TODO: The following fields are work in progress:

# YOLO confidence threshold
float32 yolo_confidence_threshold

# YOLO mask threshold
float32 yolo_mask_threshold

# Segmentation mask erosion radius
uint16 segmentation_mask_erosion_radius

# Threshold for removing outlier points of segmented point cloud
float32 outlier_threshold

# YOLO models that should be enabled at this point
string[<=10] enabled_yolo_models

# YOLO classes to ignore
byte[<=256] ignored_yolo_class_indices

# FoundationPose object type (enum ordinal)
byte foundation_pose_object_type

# FoundationPose models that should be enabled at this point
byte[<=10] enabled_foundation_pose_models
