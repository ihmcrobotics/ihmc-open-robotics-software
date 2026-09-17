# Parent definition fields
behavior_msgs/ActionNodeDefinitionMessage definition

byte SETUP_OBJECT = 0
byte FREEZE_OBJECT = 1
byte DELETE_OBJECT = 2
byte CLEAR_SCENE = 3
byte FREEZE_SCENE = 4
byte CONFIGURE_PERSISTENT_DETECTIONS = 5
byte CONFIGURE_YOLO = 6
byte CONFIGURE_FOUNDATION_POSE = 7

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

# Configure persistent detections:

float32 pose_filter_alpha
float32 acceptance_confidence
float32 stability_frequency
float32 history_duration

# Configure YOLO:

# YOLO models that should be enabled at this point (by available index)
uint16[<=10] enabled_yolo_models

# Pararmeters of the enabled YOLO models
perception_msgs/YOLOv8ModelParameters[<=10] yolo_model_parameters

# Configure FoundationPose:

# FoundationPose models that should be enabled at this point
byte[<=10] enabled_foundation_pose_models
