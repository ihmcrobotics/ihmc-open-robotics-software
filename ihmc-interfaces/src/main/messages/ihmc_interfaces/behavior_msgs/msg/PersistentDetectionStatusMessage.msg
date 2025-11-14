# Status of a persistent detection

# Last 4 digits of UUID as a string
string id

# Instant detection simple class name (e.g. YOLO, FoundationPose)
string detection_type

# Object class name (e.g., "bottle")
string object_class

# Detection frequency (Hz) with decay
float64 decaying_frequency

# Number of detections in history
int32 history_size

# Whether the detection is stable
bool is_stable

# Alpha filtered transform to world
controller_msgs/RigidBodyTransformMessage transform_to_world

# Alpha filtered transform to camera
controller_msgs/RigidBodyTransformMessage transform_to_camera
