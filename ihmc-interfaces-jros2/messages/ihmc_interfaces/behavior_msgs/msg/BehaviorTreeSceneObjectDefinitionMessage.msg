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

# Composite Reference Frame:

# Composite frame name
string composite_frame_name

# The name of frame A
string composite_frame_a

# The name of frame B
string composite_frame_b

# From A towards, backed up by X distance, Zup
byte COMPOSITE_FRAME_APPROACH = 0
byte COMPOSITE_FRAME_HYBRID = 1

# The type of composite frame
byte composite_frame_type

# Distance from frame B
float32 composite_distance
