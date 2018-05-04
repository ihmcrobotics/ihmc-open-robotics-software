
byte VIDEO_SOURCE_MULTISENSE_LEFT_EYE = 0
byte VIDEO_SOURCE_MULTISENSE_RIGHT_EYE = 1
byte VIDEO_SOURCE_FISHEYE_LEFT = 2
byte VIDEO_SOURCE_FISHEYE_RIGHT = 3
byte VIDEO_SOURCE_CV_THRESHOLD = 4
byte VIDEO_SOURCE_IMAGE_PROCESSING_BEHAVIOR = 5
byte VIDEO_SOURCE_AWARE_FACE_TRACKER = 6

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

byte video_source 255
int64 timestamp
byte[] data
geometry_msgs/Point position
geometry_msgs/Quaternion orientation
IntrinsicParametersMessage intrinsic_parameters