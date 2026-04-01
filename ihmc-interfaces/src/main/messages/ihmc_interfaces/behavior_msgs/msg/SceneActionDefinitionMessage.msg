# Parent definition fields
behavior_msgs/ActionNodeDefinitionMessage definition

byte SETUP_OBJECT = 0
byte FREEZE_OBJECT = 1
byte DELETE_OBJECT = 2
byte CLEAR_SCENE = 3
byte FREEZE_SCENE = 4
byte CONFIGURE_YOLO = 5
byte CONFIGURE_FOUNDATION_POSE = 5
byte CREATE_FRAME = 7

# The type of action as defined above
byte scene_action_type

# Setup, freeze, delete object:

# Definition of the object to setup
behavior_msgs/BehaviorTreeSceneObjectDefinitionMessage scene_object_definition

# If this action can't do what it needs to do in this amount of time, give up.
float32 timeout

# Filter for persistent detection history size
uint16 minimum_history_size

# The nominal object pose for behavior previewing
controller_msgs/RigidBodyTransformMessage nominal_object_pose

# Configure YOLO: TODO: The following fields are work in progress:

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

# Configure FoundationPose:

# FoundationPose models that should be enabled at this point
byte[<=10] enabled_foundation_pose_models

# CREATE_FRAME: Creates a Z-up from where X points to frame B from frame A and is some distance from frame B

# The name of the frame to create
string frame_name

# The name of frame A
string frame_a

# The name of frame B
string frame_b

# Distance from frame B
float32 distance
