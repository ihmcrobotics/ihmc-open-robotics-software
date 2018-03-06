
byte VIDEO_SOURCE_MULTISENSE_LEFT_EYE = 0
byte VIDEO_SOURCE_MULTISENSE_RIGHT_EYE = 1
byte VIDEO_SOURCE_FISHEYE_LEFT = 2
byte VIDEO_SOURCE_FISHEYE_RIGHT = 3
byte VIDEO_SOURCE_CV_THRESHOLD = 4
byte VIDEO_SOURCE_IMAGE_PROCESSING_BEHAVIOR = 5
byte VIDEO_SOURCE_AWARE_FACE_TRACKER = 6

byte video_source -1
int64 timestamp
byte[] data
geometry_msgs/Point position
geometry_msgs/Quaternion orientation
IntrinsicParametersMessage intrinsicParameters