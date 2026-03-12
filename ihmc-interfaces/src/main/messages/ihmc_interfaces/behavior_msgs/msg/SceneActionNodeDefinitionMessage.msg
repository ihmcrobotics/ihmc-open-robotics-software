# Parent definition fields
behavior_msgs/ActionNodeDefinitionMessage definition

byte SETUP_OBJECT = 0
byte FREEZE_OBJECT = 1
byte DELETE_OBJECT = 2

# The type of action as defined above
byte scene_action_type

# Definition of the object to setup
behavior_msgs/BehaviorTreeSceneObjectDefinitionMessage scene_object_definition

# If this action can't do what it needs to do in this amount of time, give up.
float32 timeout

# Filter for persistent detection history size
uint16 minimum_history_size

# The nominal object pose for behavior previewing
geometry_msgs/Pose nominal_object_pose

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
