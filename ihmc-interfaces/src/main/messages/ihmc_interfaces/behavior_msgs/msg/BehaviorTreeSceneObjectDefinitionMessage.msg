# Object type: YOLO only, FoundationPose, custom frame, or specific heurustic
byte object_type

# YOLO:

# YOLO model name to use
string yolo_model_name

# Class name to use for the object
string yolo_class_name

# FoundationPose:

# FoundationPose object type (IsaacROSFoundationPoseObject enum ordinal)
byte foundation_pose_object_type

# Door Frame

# Minimum number of points to detect the post
uint16 min_post_points

# Minimum number of point to detect the door recess
uint16 min_recess_points

# Custom Reference Frame: Creates a Z-up from where X points to frame B from frame A and is some distance from frame B

# Custom frame name
string custom_frame_name

# The name of frame A
string frame_a

# The name of frame B
string frame_b

# Distance from frame B
float32 distance
